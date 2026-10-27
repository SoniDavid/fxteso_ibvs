// In-process Pi camera capture: the Y plane of YUV420, newest frame only. Replaces the image topic
// on the aircraft, where 2.2 MB frames through DDS were dropped under the rest of the stack's load.
#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace quad_control
{

struct LibcameraConfig
{
	int width = 1152;
	int height = 648;
	// Selects the sensor mode, and with it the field of view the camera preset assumes.
	int sensor_width = 2304;
	int sensor_height = 1296;
	double fps = 56.0;
	bool rotate180 = true;   // camera_node's hflip + vflip
	bool af_manual = true;
	double lens_position = 0.83;
};

class LibcameraSource
{
public:
	LibcameraSource();
	~LibcameraSource();

	// Throws std::runtime_error with the reason on any failure.
	void start(const LibcameraConfig &cfg);
	void stop();

	// Newest frame since the last call: true and a grey copy plus its capture time in
	// CLOCK_REALTIME ns (comparable with a wall-clock node->now()); false if nothing new.
	bool takeLatest(cv::Mat &grey, int64_t &stamp_realtime_ns);

	// Frames delivered by the camera, for the timing report.
	uint64_t framesCaptured() const;
	std::string description() const;

private:
	struct Impl;
	std::unique_ptr<Impl> impl_;
};

}  // namespace quad_control
