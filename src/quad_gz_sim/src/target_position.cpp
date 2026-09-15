#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include <chrono>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <string>


bool control_started = false;

float pos_x;
float pos_y;
float xp;
float yp;
float zp;
float xpp;
float ypp;
float pos_z;
float yaw;
float yawRate;
float yawAccel;
float t;
float step = 0.01;
float arg;
float x_traj;
float y_traj;

Eigen::Vector3f quad_pos;
Eigen::Vector3f error;


void quadPosCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr quadPos)
{
	quad_pos(0) = quadPos->x;
    quad_pos(1) = quadPos->y;
    quad_pos(2) = quadPos->z;
}


// ---------------------------------------------------------------------------------------
// Target velocity profiles. Each writes the globals xp, yp, yawRate (and pos_z, where it
// moves); the caller integrates them into the pose. Selected by the "profile" parameter.

// The thesis trajectory, unchanged and the default: a 0.1 -> 0.9 m/s ramp, a 94 s circle at
// 1 m/s with 0.1 rad/s of yaw and a vertical bump, then a decelerating return.
static void thesisProfile()
{
	if(t>=0 && t<2)
	{
		xp = 0;
		xp = 0;
		yawRate = 0;
	}

	else if (t >= 2 && t < 5)
	{
		xp = 0.1;
		yp = 0;
		yawRate = 0;
	}	
	else if (t >= 5 && t < 8)
	{
		xp = 0.2;
		yp = 0;
		yawRate = 0;
	}
	else if (t >= 8 && t < 11)
	{
		xp = 0.3;
		yp = 0;
		yawRate = 0;
	}
	else if (t >= 11 && t < 14)
	{
		xp = 0.4;
		yp = 0;
		yawRate = 0;
	}	
	else if (t >= 14 && t < 17)
	{
		xp = 0.5;
		yp = 0;
		yawRate = 0;
	}
	else if (t >= 17 && t < 20)
	{
		xp = 0.6;
		yp = 0;
		yawRate = 0;
	}		
	else if (t >= 20 && t < 23)
	{
		xp = 0.7;
		yp = 0;
		yawRate = 0;
	}
	else if (t >= 23 && t < 26)
	{
		xp = 0.8;
		yp = 0;
		yawRate = 0;
	}
	else if (t >= 26 && t < 30)
	{
		xp = 0.9;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=30 && t<124.2478)
	{	
		xp = 1 * cos(0.1*(t-30));
		yp = 1 * sin(0.1*(t-30));	
		yawRate = 0.1;

		if (t>=70 && t<100)
		{
			pos_z = sin(0.05*3.141592*(t-30));
		}
		else
		{
			pos_z = 0;
		}

		if (pos_z < 0)
		{
			pos_z = 0;
		}

		if (pos_z>0.5)
		{
			pos_z = 0.5;
		}

	}
	else if (t>=124.2478 && t<140)
	{
		xp = -1;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=140 && t<143)
	{
		xp = -0.8;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=143 && t<146)
	{
		xp = -0.7;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=146 && t<149)
	{
		xp = -0.6;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=149 && t<152)
	{
		xp = -0.5;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=152 && t<155)
	{
		xp = -0.4;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=155 && t<158)
	{
		xp = -0.3;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=158 && t<162)
	{
		xp = -0.2;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=158 && t<165)
	{
		xp = -0.1;
		yp = 0;
		yawRate = 0;
	}
	else if (t>=165)
	{
		xp = 0;
		yp = 0;
		yawRate = 0;
	}
	
}

// Stationary. Isolates disturbance rejection from target motion.
static void hoverProfile()
{
	xp = 0;
	yp = 0;
	yawRate = 0;
}

// Straight line ramped at `accel` to `speed` and then held, along a heading measured from +x.
// Sweeping `speed` to the point of lock loss measures the b1 of Assumption 7.
//
// `heading` exists because the camera's footprint is NOT square: at zD 1.2 module3wide_2304
// covers 2.96 m along the image's long axis and 1.67 m along its short one, and world +x maps to
// the SHORT one - so a target travelling along +x runs down the tightest direction the sensor
// has (0.62 m of slack against 1.31 m). heading=90 puts the travel on the wide axis instead,
// which is the simulation equivalent of mounting the camera rotated 90 degrees. Default 0
// reproduces every run flown before 2026-09-05.
static void lineProfile(float speed, float accel, float heading)
{
	const float t_ramp = (accel > 0) ? speed / accel : 0;
	const float v = (t < t_ramp) ? accel * t : speed;
	xp = v * cos(heading);
	yp = v * sin(heading);
	yawRate = 0;
}

// Circle at constant speed with the target yawing to match, as the thesis' own circular
// phase does; the radius is speed/yaw_rate. Sweeping `yaw_rate` measures b3.
static void circleProfile(float speed, float yaw_rate)
{
	xp = speed * cos(yaw_rate * t);
	yp = speed * sin(yaw_rate * t);
	yawRate = yaw_rate;
}

// Triangular velocity wave: accelerate at `accel` up to `speed`, decelerate back, repeat.
// Sweeping `accel` at fixed `speed` measures b2, the bound on target acceleration.
static void stepsProfile(float speed, float accel)
{
	const float t_ramp = (accel > 0) ? speed / accel : 0;
	if (t_ramp <= 0)
	{
		hoverProfile();
		return;
	}
	const float phase = fmodf(t, 2.0f * t_ramp);
	xp = (phase < t_ramp) ? accel * phase : speed - accel * (phase - t_ramp);
	yp = 0;
	yawRate = 0;
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("tgt_pos");

	fxteso::SimRate loop_rate(node, 100);
	
	auto tgt_pos_pub = node->create_publisher<geometry_msgs::msg::Vector3>("tgt_position",100);
	auto tgt_yaw_pub = node->create_publisher<std_msgs::msg::Float64>("tgt_yaw",100);
	auto tgt_vel_pub = node->create_publisher<geometry_msgs::msg::Vector3>("tgt_velocity",100);
	auto tgt_yaw_rate_pub = node->create_publisher<std_msgs::msg::Float64>("tgt_yaw_rate",100);
	auto tgt_accel_pub = node->create_publisher<geometry_msgs::msg::Vector3>("tgt_acceleration",100);
	auto tgt_yaw_acceleration_pub = node->create_publisher<std_msgs::msg::Float64>("tgt_yaw_acceleration",100);

	auto error_lin_pub = node->create_publisher<geometry_msgs::msg::Vector3>("position_error",100);
	

	auto quad_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>("quad_position", 1, quadPosCallback);

	// The trajectory assumes the aircraft is already over the marker; under px4 takeoff costs
	// ~25 s, by which time the target has driven out of frame. Default false keeps old timing.
	const bool hold_until_control =
		node->declare_parameter<bool>("hold_until_control", false);

	// Trajectory selection. "thesis" is the default and is the trajectory every recorded bag
	// was flown on; the others exist to sweep the Assumption 7 bounds and are inert until asked
	// for. speed/yaw_rate/accel are ignored by "thesis" and "hover".
	const std::string profile = node->declare_parameter<std::string>("profile", "thesis");
	const float speed = node->declare_parameter<double>("speed", 1.0);
	const float yaw_rate = node->declare_parameter<double>("yaw_rate", 0.1);
	const float accel = node->declare_parameter<double>("accel", 0.5);
	// Direction of travel for `line`, degrees from +x. 0 is the short (tight) image axis and is
	// what every pre-2026-09-05 run flew; 90 is the wide axis. See lineProfile.
	const float heading =
		node->declare_parameter<double>("heading", 0.0) * static_cast<float>(M_PI) / 180.0f;
	// Hide the target for a window to exercise the lock-loss path. Needs > 1.3 s (bridge
	// setpoint_timeout 0.3 s + COM_OF_LOSS_T 1.0 s) before PX4 fails safe. 0 disables.
	const float blackout_at = node->declare_parameter<double>("blackout_at", 0.0);
	const float blackout_for = node->declare_parameter<double>("blackout_for", 3.0);
	bool was_blacked_out = false;
	// The servoing depth /position_error is measured against. Hardcoded at 2.5 it put a
	// constant (zD - 2.5) bias on the z error of every run flown at another depth.
	const float zD = node->declare_parameter<double>("zD", 2.5);

	if (profile != "thesis" && profile != "hover" && profile != "line" &&
	    profile != "circle" && profile != "steps")
		throw std::runtime_error(
			"profile:=" + profile + " is not one of thesis, hover, line, circle, steps.");

	RCLCPP_INFO(node->get_logger(), "target profile: %s (speed %.2f m/s, yaw_rate %.2f rad/s, "
	            "accel %.2f m/s^2)", profile.c_str(), speed, yaw_rate, accel);

	auto ctrl_sub = node->create_subscription<geometry_msgs::msg::Quaternion>(
		"desired_attitude", 1,
		[](const geometry_msgs::msg::Quaternion::ConstSharedPtr) { control_started = true; });



	geometry_msgs::msg::Vector3 tgt_position;
	geometry_msgs::msg::Vector3 positionError;
	std_msgs::msg::Float64 tgt_yaw;
	
	geometry_msgs::msg::Vector3 tgt_vel;
	std_msgs::msg::Float64 tgt_yaw_vel;
	std_msgs::msg::Float64 tgt_psi_rate;

	geometry_msgs::msg::Vector3 tgt_accel;
	std_msgs::msg::Float64 tgt_yaw_accel;	
	
	
	int i = 0;
	int sim_time = 100/step; //Seconds / step

	//Tgt_pert
	pos_x = -10;
	pos_y = -10;
	//pos_x = 0;
	//pos_y = 0;
	pos_z = 0;
	yaw = 0;

	xp = 0;
	yp = 0;
	zp = 0;
	yawRate = 0;

	tgt_position.x = pos_x;
	tgt_position.y = pos_y;
	tgt_position.z = pos_z;
	
	tgt_yaw.data = yaw;
	
	tgt_vel.x = xp;
	tgt_vel.y = yp;
	tgt_vel.z = 0;

	tgt_psi_rate.data = yawRate;
	
	loop_rate.sleepFor(0.05);
	tgt_pos_pub->publish(tgt_position);
	tgt_yaw_pub->publish(tgt_yaw);
	tgt_vel_pub->publish(tgt_vel);
	tgt_yaw_rate_pub->publish(tgt_psi_rate);
	loop_rate.sleepFor(2.3);
	tgt_pos_pub->publish(tgt_position);
	tgt_yaw_pub->publish(tgt_yaw);
	tgt_vel_pub->publish(tgt_vel);
	tgt_yaw_rate_pub->publish(tgt_psi_rate);

	loop_rate.sleepFor(2.0);

	if (hold_until_control)
		RCLCPP_INFO(node->get_logger(),
		            "Holding the target on its start pose until desired_attitude appears.");

	while(rclcpp::ok())
	{
		t = i*step;
		/*
		pos_x = 0;
		pos_y = 0;
		pos_z = 0;
		yaw = 0;

		xp = 0;
		yp = 0;
		zp = 0;
		yawRate = 0;
		*/		
		/*
		xp = 0;
		yp = 0;
		yawRate = 0;
		*/
			
		if (profile == "thesis")
			thesisProfile();
		else if (profile == "hover")
			hoverProfile();
		else if (profile == "line")
			lineProfile(speed, accel, heading);
		else if (profile == "circle")
			circleProfile(speed, yaw_rate);
		else
			stepsProfile(speed, accel);
		
		pos_x = pos_x + xp * step;
		pos_y = pos_y + yp * step;
		yaw = yaw + yawRate * step;	
		

		error(0) = quad_pos(0) - pos_x;
		error(1) = quad_pos(1) - pos_y;
		error(2) = quad_pos(2) + zD + pos_z;

		// Only the PUBLISHED value moves; pos_x/pos_y keep integrating, so the target returns
		// exactly where it would have been.
		const bool blacked_out =
			blackout_at > 0.0f && t >= blackout_at && t < blackout_at + blackout_for;
		if (blacked_out != was_blacked_out)
		{
			RCLCPP_WARN(node->get_logger(), blacked_out
			            ? "BLACKOUT: hiding the target for %.1f s at t=%.1f"
			            : "blackout over at t=%.1f (%.1f s)",
			            blacked_out ? blackout_for : t, blacked_out ? t : blackout_for);
			was_blacked_out = blacked_out;
		}

		tgt_position.x = pos_x + (blacked_out ? 1000.0f : 0.0f);
		tgt_position.y = pos_y;
		tgt_position.z = -pos_z;
	
		tgt_yaw.data = yaw;

		tgt_vel.x = xp;
		tgt_vel.y = yp;
		tgt_vel.z = 0;
		tgt_psi_rate.data = yawRate;

		tgt_accel.x = 0;
		tgt_accel.y = 0;
		tgt_accel.z = 0;
		tgt_yaw_accel.data = 0;


		positionError.x = error(0);
		positionError.y = error(1);
		positionError.z = error(2);

		tgt_pos_pub->publish(tgt_position);
		tgt_yaw_pub->publish(tgt_yaw);
		
		tgt_vel_pub->publish(tgt_vel);
		tgt_yaw_rate_pub->publish(tgt_psi_rate);

		tgt_accel_pub->publish(tgt_accel);

		error_lin_pub->publish(positionError);
		

		// Freezing the clock rather than the outputs keeps the trajectory itself untouched:
		// it simply starts later, from the same t=0 it always did.
		if (!hold_until_control || control_started)
			i = i+1;

	
		loop_rate.sleep();
		
		
	}
	rclcpp::shutdown();
	return 0;	
}
