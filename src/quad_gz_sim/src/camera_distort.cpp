// Applies lens distortion to a rendered frame: gz-sim 8's ogre2 has no DistortionPass, so the
// SDF <distortion> element parses and does nothing. Warps inward, so it understates real FOV.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <vector>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("camera_distort");
	const auto log = node->get_logger();

	// Same five-element Brown-Conrady vector image_features takes, and the same preset feeds
	// both: cameras.yaml -> the launch file -> here and there.
	const std::vector<double> d =
		node->declare_parameter<std::vector<double>>("camera_distortion",
		                                            {0.0, 0.0, 0.0, 0.0, 0.0});
	const double hfov = node->declare_parameter<double>("camera_hfov", 1.085595);
	const int width = node->declare_parameter<int>("camera_width", 820);
	const int height = node->declare_parameter<int>("camera_height", 616);
	const std::string in_topic =
		node->declare_parameter<std::string>("input_topic", "/quad/camera/image_raw");
	const std::string out_topic =
		node->declare_parameter<std::string>("output_topic", "/quad/camera/image_distorted");

	if (d.size() != 5)
	{
		RCLCPP_FATAL(log, "camera_distortion needs exactly 5 elements (k1,k2,p1,p2,k3), got %zu.",
		             d.size());
		return 1;
	}

	const double fx = width / (2.0 * tan(hfov / 2.0));
	const cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, width / 2.0,
	                                             0, fx, height / 2.0,
	                                             0, 0, 1);
	const cv::Mat D = (cv::Mat_<double>(1, 5) << d[0], d[1], d[2], d[3], d[4]);

	// Inverted on purpose: for each DISTORTED output pixel the source is where the ideal render
	// put that ray, so it is the output grid that gets undistorted. Built once into a remap.
	std::vector<cv::Point2f> grid;
	grid.reserve(static_cast<size_t>(width) * height);
	for (int v = 0; v < height; ++v)
		for (int u = 0; u < width; ++u)
			grid.emplace_back(static_cast<float>(u), static_cast<float>(v));

	std::vector<cv::Point2f> src;
	cv::undistortPoints(grid, src, K, D, cv::noArray(), K);

	cv::Mat map_x(height, width, CV_32FC1), map_y(height, width, CV_32FC1);
	for (int v = 0; v < height; ++v)
		for (int u = 0; u < width; ++u)
		{
			const cv::Point2f &p = src[static_cast<size_t>(v) * width + u];
			map_x.at<float>(v, u) = p.x;
			map_y.at<float>(v, u) = p.y;
		}

	const bool active = d[0] != 0.0 || d[1] != 0.0 || d[2] != 0.0 || d[3] != 0.0 || d[4] != 0.0;
	RCLCPP_INFO(log, "%dx%d, fx %.1f px, k=[%g %g %g] p=[%g %g] -> %s%s",
	            width, height, fx, d[0], d[1], d[4], d[2], d[3], out_topic.c_str(),
	            active ? "" : "  (all zero: passing frames through unchanged)");

	auto pub = node->create_publisher<sensor_msgs::msg::Image>(out_topic, 1);
	auto sub = node->create_subscription<sensor_msgs::msg::Image>(
		in_topic, 1,
		[&](const sensor_msgs::msg::Image::ConstSharedPtr msg)
		{
			if (!active)
			{
				pub->publish(*msg);
				return;
			}
			// BORDER_CONSTANT, not REPLICATE: a wide lens leaves real corners unimaged, and
			// smearing the edge pixels into them would invent contrast the detector could
			// latch onto. Black is what an over-wide sensor actually sees there.
			cv_bridge::CvImageConstPtr in;
			try
			{
				in = cv_bridge::toCvShare(msg, msg->encoding);
			}
			catch (const cv_bridge::Exception &e)
			{
				RCLCPP_ERROR_THROTTLE(log, *node->get_clock(), 5000,
				                      "cv_bridge: %s", e.what());
				return;
			}
			if (in->image.cols != width || in->image.rows != height)
			{
				RCLCPP_ERROR_THROTTLE(
					log, *node->get_clock(), 5000,
					"image is %dx%d but the map was built for %dx%d - camera_width/height "
					"disagree with what is being rendered.",
					in->image.cols, in->image.rows, width, height);
				return;
			}
			cv::Mat out;
			cv::remap(in->image, out, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
			          cv::Scalar(0, 0, 0));
			pub->publish(*cv_bridge::CvImage(msg->header, msg->encoding, out).toImageMsg());
		});

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
