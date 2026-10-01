// PX4's offboard-loss Hold for the plants without PX4: last setpoint kept 0.3 s + COM_OF_LOSS_T,
// then PX4's position cascade (fork defaults) on the plant state, until pos_ctrl publishes again.
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <cmath>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("sim_hold");

	const double mass = node->declare_parameter<double>("quad_mass", 2.0);
	const double g = 9.81;
	// px4_offboard_bridge's setpoint_timeout, then the airframe's COM_OF_LOSS_T.
	const double setpoint_timeout = node->declare_parameter<double>("setpoint_timeout", 0.3);
	const double loss_timeout = node->declare_parameter<double>("loss_timeout", 1.0);
	// MPC_XY_P, MPC_Z_P, MPC_XY_VEL_{P,I,D}_ACC, MPC_Z_VEL_{P,I}_ACC, MPC_XY_VEL_MAX,
	// MPC_Z_VEL_MAX_{UP,DN}, MPC_TILTMAX_AIR: the PX4 fork's defaults.
	const double xy_p = node->declare_parameter<double>("xy_p", 0.95);
	const double z_p = node->declare_parameter<double>("z_p", 1.0);
	const double xy_vp = node->declare_parameter<double>("xy_vel_p", 1.8);
	const double xy_vi = node->declare_parameter<double>("xy_vel_i", 0.4);
	const double xy_vd = node->declare_parameter<double>("xy_vel_d", 0.2);
	const double z_vp = node->declare_parameter<double>("z_vel_p", 4.0);
	const double z_vi = node->declare_parameter<double>("z_vel_i", 2.0);
	const double xy_vmax = node->declare_parameter<double>("xy_vel_max", 12.0);
	const double z_vmax_up = node->declare_parameter<double>("z_vel_max_up", 3.0);
	const double z_vmax_dn = node->declare_parameter<double>("z_vel_max_dn", 1.5);
	const double tilt_max = node->declare_parameter<double>("tilt_max_deg", 45.0) * M_PI / 180.0;
	// MPC_VELD_LP: the velocity-derivative low-pass cutoff.
	const double veld_lp_hz = node->declare_parameter<double>("vel_deriv_lp_hz", 5.0);

	Eigen::Vector3d pos(0, 0, 0), vel(0, 0, 0), att(0, 0, 0);
	bool have_state = false;
	double last_yaw_cmd = 0.0;
	bool loop_started = false;
	rclcpp::Time last_ctrl(0, 0, RCL_ROS_TIME);
	bool holding = false;

	auto ctrl_sub = node->create_subscription<geometry_msgs::msg::Quaternion>(
		"ibvs_control_input", 10, [&](geometry_msgs::msg::Quaternion::ConstSharedPtr) {
			last_ctrl = node->now();
			loop_started = true;
		});
	// pos_ctrl's own yaw setpoint, so Hold keeps the heading the loop last asked for.
	auto att_des_sub = node->create_subscription<geometry_msgs::msg::Quaternion>(
		"desired_attitude", 10, [&](geometry_msgs::msg::Quaternion::ConstSharedPtr m) {
			if (!holding)
				last_yaw_cmd = m->z;
		});
	auto pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_position", 10, [&](geometry_msgs::msg::Vector3::ConstSharedPtr m) {
			pos << m->x, m->y, m->z;
			have_state = true;
		});
	auto vel_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_velocity", 10, [&](geometry_msgs::msg::Vector3::ConstSharedPtr m) {
			vel << m->x, m->y, m->z;
		});
	auto att_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_attitude", 10, [&](geometry_msgs::msg::Vector3::ConstSharedPtr m) {
			att << m->x, m->y, m->z;
		});

	auto att_pub = node->create_publisher<geometry_msgs::msg::Quaternion>("desired_attitude", 10);
	auto thrust_pub = node->create_publisher<std_msgs::msg::Float64>("quad_thrust", 10);
	auto active_pub = node->create_publisher<std_msgs::msg::Bool>("sim_hold/active", 10);

	const double rate = 50.0, dt = 1.0 / rate;
	fxteso::SimRate loop_rate(node, rate);
	Eigen::Vector3d hold_pos(0, 0, 0), vel_int(0, 0, 0), vel_prev(0, 0, 0), vel_deriv(0, 0, 0);
	const double lp_alpha = dt / (dt + 1.0 / (2.0 * M_PI * veld_lp_hz));

	RCLCPP_INFO(node->get_logger(),
	            "PX4 Hold emulation: last setpoint kept %.1f s after a %.1f s stale stream, "
	            "then hold at %.2f kg", loss_timeout, setpoint_timeout, mass);

	while (rclcpp::ok())
	{
		const double silent = loop_started ? (node->now() - last_ctrl).seconds() : 0.0;
		const bool want = loop_started && have_state && silent > setpoint_timeout + loss_timeout;

		if (want && !holding)
		{
			hold_pos = pos;
			vel_int.setZero();
			vel_prev = vel;
			vel_deriv.setZero();
			RCLCPP_WARN(node->get_logger(),
			            "pos_ctrl silent %.2f s - holding at (%.2f, %.2f, %.2f) NED", silent,
			            hold_pos(0), hold_pos(1), hold_pos(2));
		}
		else if (!want && holding)
		{
			RCLCPP_INFO(node->get_logger(), "pos_ctrl publishing again - hold released");
		}
		holding = want;

		if (holding)
		{
			// Position P -> velocity setpoint, limited as PX4 limits it.
			Eigen::Vector3d vel_sp(xy_p * (hold_pos(0) - pos(0)), xy_p * (hold_pos(1) - pos(1)),
			                       z_p * (hold_pos(2) - pos(2)));
			const double vxy = std::hypot(vel_sp(0), vel_sp(1));
			if (vxy > xy_vmax)
				vel_sp.head<2>() *= xy_vmax / vxy;
			vel_sp(2) = std::clamp(vel_sp(2), -z_vmax_up, z_vmax_dn);

			// Velocity PID -> acceleration setpoint (NED); D acts on the filtered measured
			// velocity derivative, as in PX4's PositionControl.
			vel_deriv += lp_alpha * ((vel - vel_prev) / dt - vel_deriv);
			vel_prev = vel;
			const Eigen::Vector3d e = vel_sp - vel;
			vel_int = (vel_int + e * dt).cwiseMax(-2.0).cwiseMin(2.0);
			Eigen::Vector3d acc(xy_vp * e(0) + xy_vi * vel_int(0) - xy_vd * vel_deriv(0),
			                    xy_vp * e(1) + xy_vi * vel_int(1) - xy_vd * vel_deriv(1),
			                    z_vp * e(2) + z_vi * vel_int(2));

			// Tilt limit: horizontal demand may not lean the thrust past tilt_max.
			const double lift = std::max(g - acc(2), 0.1 * g);
			const double axy = acc.head<2>().norm(), axy_max = lift * std::tan(tilt_max);
			if (axy > axy_max)
				acc.head<2>() *= axy_max / axy;

			// pos_ctrl's own mapping: F = m a in the yaw frame, T b3 = m g e3 - F.
			const double yaw = att(2);
			Eigen::Matrix3d Rz;
			Rz << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
			const Eigen::Vector3d F = mass * (Rz.transpose() * acc);
			Eigen::Matrix3d Rp, Rr;
			Rp << cos(att(1)), 0, sin(att(1)), 0, 1, 0, -sin(att(1)), 0, cos(att(1));
			Rr << 1, 0, 0, 0, cos(att(0)), -sin(att(0)), 0, sin(att(0)), cos(att(0));
			const Eigen::Vector3d e3(0, 0, 1);
			const double thrust = std::max(
				0.0, e3.dot((Rp * Rr).transpose() * (mass * g * e3 - F)));
			const double roll = asin(std::clamp(F(1) / std::max(thrust, 1e-3), -1.0, 1.0));
			const double pitch = asin(std::clamp(
				-F(0) / std::max(thrust * cos(roll), 1e-3), -1.0, 1.0));

			geometry_msgs::msg::Quaternion a;
			a.x = roll;
			a.y = pitch;
			a.z = last_yaw_cmd;
			a.w = 0.0;
			att_pub->publish(a);
			std_msgs::msg::Float64 t;
			t.data = thrust;
			thrust_pub->publish(t);
		}

		std_msgs::msg::Bool active;
		active.data = holding;
		active_pub->publish(active);
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}
