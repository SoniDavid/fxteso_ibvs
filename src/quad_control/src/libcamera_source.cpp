#include "quad_control/libcamera_source.hpp"

#include <libcamera/libcamera.h>

#include <sys/mman.h>
#include <time.h>

#include <atomic>
#include <cstring>
#include <map>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace quad_control
{

namespace
{
int64_t clockNs(clockid_t id)
{
	timespec ts;
	clock_gettime(id, &ts);
	return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
}  // namespace

struct LibcameraSource::Impl
{
	std::unique_ptr<libcamera::CameraManager> manager;
	std::shared_ptr<libcamera::Camera> camera;
	std::unique_ptr<libcamera::CameraConfiguration> config;
	std::unique_ptr<libcamera::FrameBufferAllocator> allocator;
	std::vector<std::unique_ptr<libcamera::Request>> requests;
	libcamera::Stream *stream = nullptr;
	// Y plane of each buffer, mapped once.
	std::map<const libcamera::FrameBuffer *, const uint8_t *> y_plane;
	std::vector<std::pair<void *, size_t>> mappings;
	unsigned int stride = 0;
	int width = 0;
	int height = 0;
	bool started = false;
	bool acquired = false;
	std::string desc;

	std::mutex mtx;
	cv::Mat latest;
	int64_t latest_stamp = 0;
	bool fresh = false;
	std::atomic<uint64_t> captured{0};
	// Cleared before stop(): a request completing during the stop must not be requeued.
	std::atomic<bool> running{false};

	void onRequestCompleted(libcamera::Request *request)
	{
		if (request->status() == libcamera::Request::RequestCancelled || !running)
			return;
		const libcamera::FrameBuffer *buffer = request->buffers().begin()->second;
		// The sensor timestamp is CLOCK_MONOTONIC (V4L2); shift it onto the wall clock.
		const int64_t sensor_ns = static_cast<int64_t>(buffer->metadata().timestamp);
		const int64_t stamp = clockNs(CLOCK_REALTIME) - (clockNs(CLOCK_MONOTONIC) - sensor_ns);
		const auto it = y_plane.find(buffer);
		if (it != y_plane.end() && buffer->metadata().status == libcamera::FrameMetadata::FrameSuccess)
		{
			std::lock_guard<std::mutex> lock(mtx);
			// One row copy per line: the stride can exceed the width.
			cv::Mat(height, width, CV_8UC1, const_cast<uint8_t *>(it->second), stride).copyTo(latest);
			latest_stamp = stamp;
			fresh = true;
			++captured;
		}
		request->reuse(libcamera::Request::ReuseBuffers);
		if (running)
			camera->queueRequest(request);
	}
};

LibcameraSource::LibcameraSource() : impl_(std::make_unique<Impl>()) {}

LibcameraSource::~LibcameraSource()
{
	stop();
}

void LibcameraSource::start(const LibcameraConfig &cfg)
{
	using namespace libcamera;
	Impl &d = *impl_;

	d.manager = std::make_unique<CameraManager>();
	if (d.manager->start())
		throw std::runtime_error("libcamera CameraManager failed to start");
	if (d.manager->cameras().empty())
		throw std::runtime_error("libcamera found no camera");
	d.camera = d.manager->cameras().front();
	if (d.camera->acquire())
		throw std::runtime_error("camera " + d.camera->id() + " is busy (is camera_node running?)");
	d.acquired = true;

	d.config = d.camera->generateConfiguration({StreamRole::VideoRecording});
	if (!d.config)
		throw std::runtime_error("libcamera could not generate a video configuration");
	StreamConfiguration &sc = d.config->at(0);
	sc.pixelFormat = formats::YUV420;
	sc.size = Size(cfg.width, cfg.height);
	sc.bufferCount = 6;
	d.config->orientation = cfg.rotate180 ? Orientation::Rotate180 : Orientation::Rotate0;
	SensorConfiguration sensor;
	sensor.outputSize = Size(cfg.sensor_width, cfg.sensor_height);
	sensor.bitDepth = 10;
	d.config->sensorConfig = sensor;

	const CameraConfiguration::Status status = d.config->validate();
	if (status == CameraConfiguration::Invalid)
		throw std::runtime_error("libcamera rejected the configuration " + sc.toString());
	// Adjusted is not acceptable: the feature model and aD assume exactly this size.
	if (sc.size != Size(cfg.width, cfg.height) || sc.pixelFormat != formats::YUV420)
		throw std::runtime_error("libcamera adjusted the stream to " + sc.toString() +
		                         "; camera_width/height must match what it can produce");
	if (d.camera->configure(d.config.get()))
		throw std::runtime_error("libcamera failed to configure " + sc.toString());

	d.stream = sc.stream();
	d.stride = sc.stride;
	d.width = cfg.width;
	d.height = cfg.height;

	d.allocator = std::make_unique<FrameBufferAllocator>(d.camera);
	if (d.allocator->allocate(d.stream) < 0)
		throw std::runtime_error("libcamera failed to allocate buffers");

	for (const std::unique_ptr<FrameBuffer> &buffer : d.allocator->buffers(d.stream))
	{
		const FrameBuffer::Plane y = buffer->planes()[0];
		// Planes can share one dmabuf at different offsets: map from 0 to the plane's end.
		const size_t len = y.offset + y.length;
		void *mem = mmap(nullptr, len, PROT_READ, MAP_SHARED, y.fd.get(), 0);
		if (mem == MAP_FAILED)
			throw std::runtime_error("mmap of a camera buffer failed");
		d.mappings.emplace_back(mem, len);
		d.y_plane[buffer.get()] = static_cast<const uint8_t *>(mem) + y.offset;

		std::unique_ptr<Request> request = d.camera->createRequest();
		if (!request || request->addBuffer(d.stream, buffer.get()))
			throw std::runtime_error("libcamera failed to create a request");
		d.requests.push_back(std::move(request));
	}

	d.camera->requestCompleted.connect(&d, &Impl::onRequestCompleted);

	ControlList controls(controls::controls);
	const int64_t frame_us = static_cast<int64_t>(1e6 / cfg.fps);
	const int64_t limits[2] = {frame_us, frame_us};
	controls.set(controls::FrameDurationLimits, Span<const int64_t, 2>(limits));
	if (cfg.af_manual)
	{
		controls.set(controls::AfMode, controls::AfModeManual);
		controls.set(controls::LensPosition, static_cast<float>(cfg.lens_position));
	}
	else
	{
		controls.set(controls::AfMode, controls::AfModeContinuous);
	}

	if (d.camera->start(&controls))
		throw std::runtime_error("libcamera failed to start the camera");
	d.started = true;
	d.running = true;
	for (std::unique_ptr<Request> &request : d.requests)
		d.camera->queueRequest(request.get());

	d.desc = d.camera->id() + " " + sc.toString() + ", sensor " +
	         std::to_string(cfg.sensor_width) + "x" + std::to_string(cfg.sensor_height) +
	         ", stride " + std::to_string(d.stride) + ", " + std::to_string(frame_us) + " us/frame" +
	         (cfg.rotate180 ? ", rotated 180" : "") +
	         (cfg.af_manual ? ", lens " + std::to_string(cfg.lens_position) : ", continuous AF");
}

void LibcameraSource::stop()
{
	Impl &d = *impl_;
	d.running = false;
	if (d.started)
	{
		d.camera->stop();
		d.started = false;
	}
	if (d.camera)
		d.camera->requestCompleted.disconnect(&d);
	d.requests.clear();
	for (auto &m : d.mappings)
		munmap(m.first, m.second);
	d.mappings.clear();
	d.y_plane.clear();
	if (d.allocator && d.stream)
		d.allocator->free(d.stream);
	d.allocator.reset();
	if (d.acquired)
	{
		d.camera->release();
		d.acquired = false;
	}
	d.config.reset();
	d.camera.reset();
	if (d.manager)
	{
		d.manager->stop();
		d.manager.reset();
	}
}

bool LibcameraSource::takeLatest(cv::Mat &grey, int64_t &stamp_realtime_ns)
{
	Impl &d = *impl_;
	std::lock_guard<std::mutex> lock(d.mtx);
	if (!d.fresh)
		return false;
	d.fresh = false;
	// Swap, not copy: the callback's next frame lands in a new buffer of its own.
	cv::swap(grey, d.latest);
	d.latest.release();
	stamp_realtime_ns = d.latest_stamp;
	return true;
}

uint64_t LibcameraSource::framesCaptured() const
{
	return impl_->captured.load();
}

std::string LibcameraSource::description() const
{
	return impl_->desc;
}

}  // namespace quad_control
