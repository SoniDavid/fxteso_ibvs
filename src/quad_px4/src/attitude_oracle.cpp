// SITL-only diagnostic: publishes a reference attitude built from Gazebo ground-truth roll/pitch
// and EKF2's yaw, so the loop's two attitude paths can be fed truth one at a time.
//   attitude_reference        PX4 NED/FRD quaternion, for px4_offboard_bridge's setpoint correction
//   quad_attitude_reference   /quad_attitude's convention, for a second td_attitude -> derotation
// replay_csv adds a recorded (t, roll_err, pitch_err) series to the truth, t = 0 at the handover.
#include <rclcpp/rclcpp.hpp>
#include "quad_common/unwrapped.hpp"
#include "quad_px4/px4_topic.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>

static constexpr double DEG = M_PI / 180.0;

struct Sample { double t, roll, pitch; };

static std::vector<Sample> loadReplay(const std::string &path, rclcpp::Logger log)
{
	std::vector<Sample> out;
	std::ifstream in(path);
	if (!in)
	{
		RCLCPP_FATAL(log, "replay_csv %s cannot be opened.", path.c_str());
		return out;
	}
	std::string line;
	while (std::getline(in, line))
	{
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream ss(line);
		Sample s;
		if (ss >> s.t >> s.roll >> s.pitch)
			out.push_back(s);
	}
	return out;
}

// Linear interpolation, looping past the end so a replay never runs out mid-flight.
static void replayAt(const std::vector<Sample> &r, double t, double &roll, double &pitch)
{
	const double span = r.back().t - r.front().t;
	if (span > 0.0)
		t = r.front().t + std::fmod(std::max(t, 0.0), span);
	auto hi = std::lower_bound(r.begin(), r.end(), t,
	                           [](const Sample &s, double v) { return s.t < v; });
	if (hi == r.begin() || hi == r.end())
	{
		const Sample &s = hi == r.end() ? r.back() : r.front();
		roll = s.roll;
		pitch = s.pitch;
		return;
	}
	const Sample &a = *(hi - 1), &b = *hi;
	const double w = (t - a.t) / std::max(b.t - a.t, 1e-9);
	roll = a.roll + w * (b.roll - a.roll);
	pitch = a.pitch + w * (b.pitch - a.pitch);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("attitude_oracle");
	auto log = node->get_logger();

	const std::string replay_csv = node->declare_parameter<std::string>("replay_csv", "");
	// Refuse to start unless truth and EKF2 agree this well while at rest: a frame error is
	// tens of degrees, EKF2's resting tilt error a fraction of one.
	const double max_tilt_disagreement = node->declare_parameter<double>("max_tilt_disagreement", 2.0);
	const double max_yaw_disagreement = node->declare_parameter<double>("max_yaw_disagreement", 30.0);
	const double check_seconds = node->declare_parameter<double>("check_seconds", 2.0);

	std::vector<Sample> replay;
	if (!replay_csv.empty())
	{
		replay = loadReplay(replay_csv, log);
		if (replay.size() < 2)
		{
			RCLCPP_FATAL(log, "replay_csv %s has %zu usable rows; need (t, roll_err, pitch_err).",
			             replay_csv.c_str(), replay.size());
			return 1;
		}
	}

	// gz world is ENU with FLU body axes; PX4 is NED/FRD. Same rotations as vicon_px4_bridge.
	const tf2::Quaternion qWorld(tf2::Vector3(M_SQRT1_2, M_SQRT1_2, 0.0), M_PI);
	const tf2::Quaternion qBody(tf2::Vector3(1.0, 0.0, 0.0), M_PI);

	auto refPub = node->create_publisher<geometry_msgs::msg::Quaternion>("attitude_reference", 10);
	auto refRpyPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_attitude_reference", 10);
	auto verifiedPub = node->create_publisher<std_msgs::msg::Bool>(
		"~/verified", rclcpp::QoS(1).transient_local());

	tf2::Quaternion qEkf;
	bool have_ekf = false;
	double yaw_ws = 0.0;
	bool have_yaw_ws = false;
	fxteso::Unwrapped unwrapRoll, unwrapPitch;

	bool verified = false;
	double check_start = -1.0;
	std::vector<double> tilt_diffs;
	double t0 = -1.0;

	const rclcpp::QoS px4Qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
	auto ekfSub = node->create_subscription<px4_msgs::msg::VehicleOdometry>(
		quad_px4::px4Topic<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry"), px4Qos,
		[&](const px4_msgs::msg::VehicleOdometry::ConstSharedPtr o)
		{
			if (!std::isfinite(o->q[0]))
				return;
			qEkf = tf2::Quaternion(o->q[1], o->q[2], o->q[3], o->q[0]);
			have_ekf = true;
		});

	// Yaw in the workspace datum, exactly as the loop sees it.
	auto yawSub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_attitude", 10,
		[&](const geometry_msgs::msg::Vector3::ConstSharedPtr a)
		{
			yaw_ws = a->z;
			have_yaw_ws = true;
		});

	// The replay clock starts when the bridge claims the aircraft, matching how it was recorded.
	bool offboard_requested = false;
	auto claimSub = node->create_subscription<std_msgs::msg::Bool>(
		"/px4_offboard_bridge/offboard_requested", rclcpp::QoS(1).transient_local(),
		[&](const std_msgs::msg::Bool::ConstSharedPtr b) { offboard_requested = b->data; });

	auto truthSub = node->create_subscription<nav_msgs::msg::Odometry>(
		"/quad_state", 10,
		[&](const nav_msgs::msg::Odometry::ConstSharedPtr s)
		{
			if (!have_ekf || !have_yaw_ws)
				return;
			const auto &o = s->pose.pose.orientation;
			const tf2::Quaternion qTrue = qWorld * tf2::Quaternion(o.x, o.y, o.z, o.w) * qBody;
			double rT, pT, yT, rE, pE, yE;
			tf2::Matrix3x3(qTrue).getRPY(rT, pT, yT);
			tf2::Matrix3x3(qEkf).getRPY(rE, pE, yE);
			const double now = node->now().seconds();

			if (!verified)
			{
				const tf2::Vector3 zT = tf2::quatRotate(qTrue, tf2::Vector3(0, 0, 1));
				const tf2::Vector3 zE = tf2::quatRotate(qEkf, tf2::Vector3(0, 0, 1));
				const double tilt = std::acos(std::clamp(zT.dot(zE), -1.0, 1.0)) / DEG;
				const double yaw = std::fabs(std::remainder(yT - yE, 2.0 * M_PI)) / DEG;
				if (check_start < 0.0)
					check_start = now;
				tilt_diffs.push_back(tilt);
				if (yaw > max_yaw_disagreement)
				{
					RCLCPP_FATAL(log, "truth and EKF2 disagree by %.1f deg in YAW - the ENU->NED "
					             "conversion is wrong for this world. Refusing to publish.", yaw);
					rclcpp::shutdown();
					return;
				}
				if (now - check_start < check_seconds)
					return;
				std::sort(tilt_diffs.begin(), tilt_diffs.end());
				const double med = tilt_diffs[tilt_diffs.size() / 2];
				if (med > max_tilt_disagreement)
				{
					RCLCPP_FATAL(log, "truth and EKF2 disagree by %.2f deg in TILT (median of %zu) "
					             "- a body-frame error (FLU vs FRD) or a moving aircraft. Refusing "
					             "to publish.", med, tilt_diffs.size());
					rclcpp::shutdown();
					return;
				}
				verified = true;
				std_msgs::msg::Bool b;
				b.data = true;
				verifiedPub->publish(b);
				RCLCPP_INFO(log, "verified: truth vs EKF2 tilt %.2f deg, yaw %.1f deg. Publishing "
				            "the reference attitude%s.", med, yaw,
				            replay.empty() ? "" : " WITH the replayed error series");
			}

			double er = 0.0, ep = 0.0;
			if (!replay.empty() && (offboard_requested || t0 >= 0.0))
			{
				if (t0 < 0.0)
					t0 = now;
				replayAt(replay, now - t0, er, ep);
			}
			const double roll = rT + er, pitch = pT + ep;

			tf2::Quaternion qRef;
			qRef.setRPY(roll, pitch, yE);
			geometry_msgs::msg::Quaternion q;
			q.x = qRef.x();
			q.y = qRef.y();
			q.z = qRef.z();
			q.w = qRef.w();
			refPub->publish(q);

			geometry_msgs::msg::Vector3 v;
			v.x = unwrapRoll(roll);
			v.y = unwrapPitch(pitch);
			v.z = yaw_ws;
			refRpyPub->publish(v);
		});

	RCLCPP_WARN(log, "attitude_oracle: GROUND TRUTH enters the loop through this node. SITL "
	            "diagnostic only - never a scored or deployable configuration.");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
