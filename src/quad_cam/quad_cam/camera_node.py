"""Raspberry Pi Camera Module 3 Wide publisher for the IBVS stack.

Publishes `sensor_msgs/Image` (bgr8) on `/quad/camera/image_raw` at the sensor mode's
full rate (~56 Hz), ISP-downscaled to the sim's `module3wide_2304` render size
(1152x648) so `image_features`' `fx`, `aD` and distortion model carry over from
simulation unchanged.

Backend is picamera2 (the RaspberryPi libcamera fork + PiSP stack, built to /usr/local
by tools/setup_pi5_ubuntu_camera.sh). picamera2 has no rosdep key and is NOT declared in
package.xml - it is satisfied out-of-band and resolves because ROS Jazzy runs on the
system python3 the /usr/local .pth files bridge onto.

Reliability pattern (from TE3002B_mcrChallenge/pzb_camera/camera_raw_publisher.py): the
capture thread only drops the newest frame into a depth-1 queue and returns; a dedicated
publisher thread builds and publishes the message. A slow or stalled subscriber therefore
drops frames (latest wins) instead of back-pressuring capture. `Image.data` is filled from
an `array.array` to hit rclpy's buffer fast-path rather than its per-byte loop.

`use_sim_time` MUST be false on the aircraft - there is no /clock and the stamp here is
the ROS system clock at the instant the frame is delivered.
"""

import array
import os
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image

from picamera2 import Picamera2
from libcamera import Transform, controls


def _image_qos(kind: str) -> QoSProfile:
    """KEEP_LAST depth-1; RELIABLE or BEST_EFFORT.

    `image_features` currently subscribes RELIABLE depth-1, so `reliable` is the drop-in
    default. `sensor_data` (BEST_EFFORT) is the correct streaming-sensor choice and is
    what a future `image_features` `image_qos` parameter should select - a RELIABLE
    subscription to a BEST_EFFORT publisher receives nothing, silently.
    """
    reliability = (QoSReliabilityPolicy.BEST_EFFORT if kind == "sensor_data"
                   else QoSReliabilityPolicy.RELIABLE)
    return QoSProfile(
        reliability=reliability,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")

        # -- parameters ------------------------------------------------------------------
        self._width = int(self.declare_parameter("width", 1152).value)
        self._height = int(self.declare_parameter("height", 648).value)
        sensor_w = int(self.declare_parameter("sensor_width", 2304).value)
        sensor_h = int(self.declare_parameter("sensor_height", 1296).value)
        # 0.0 -> run at the sensor mode's own max rate. Deliberately faster than the 50 Hz
        # image_features loop so the depth-1 consumer always has a fresh frame (age
        # 0-18 ms) rather than beating against an equal-rate source.
        framerate = float(self.declare_parameter("framerate", 0.0).value)
        image_topic = str(self.declare_parameter("image_topic",
                                                 "/quad/camera/image_raw").value)
        image_qos = str(self.declare_parameter("image_qos", "reliable").value)
        self._frame_id = str(self.declare_parameter("frame_id",
                                                    "camera_optical_frame").value)
        hflip = bool(self.declare_parameter("hflip", True).value)
        vflip = bool(self.declare_parameter("vflip", True).value)
        af_mode = str(self.declare_parameter("af_mode", "continuous").value)
        lens_position = float(self.declare_parameter("lens_position", 0.83).value)
        publish_camera_info = bool(
            self.declare_parameter("publish_camera_info", False).value)
        camera_info_file = str(self.declare_parameter("camera_info_file", "").value)
        camera_info_topic = str(self.declare_parameter("camera_info_topic",
                                                       "/quad/camera/camera_info").value)
        self._watchdog_timeout_s = float(
            self.declare_parameter("watchdog_timeout_s", 0.5).value)
        # consecutive capture failures before giving up and logging FATAL
        self._max_failures = int(self.declare_parameter("max_failures", 25).value)
        # Minimum wall-clock time between restart *attempts*, independent of how many
        # timeouts occur in between. A stop/start is a real fix for a wedged camera, but
        # under CPU contention (a busy subscriber pegging a core) capture_request() can
        # miss its schedule slot without the camera being wedged at all - restarting on a
        # fixed failure COUNT then fires roughly every failures_before_restart *
        # watchdog_timeout_s seconds (2.5 s at the defaults) for as long as the contention
        # lasts, and each restart (stop + 0.3 s + start + AE resettle) adds load of its
        # own, compounding the very contention that triggered it. Rate-limiting by time
        # bounds the worst case to one attempt per cooldown, whatever the failure rate.
        self._restart_cooldown_s = float(
            self.declare_parameter("restart_cooldown_s", 10.0).value)
        self._failures_before_restart = int(
            self.declare_parameter("failures_before_restart", 5).value)

        if image_qos not in ("reliable", "sensor_data"):
            self.get_logger().warn(
                "image_qos:=%s unknown, using 'reliable'" % image_qos)
            image_qos = "reliable"

        # -- picamera2 configuration ---------------------------------------------------
        self._picam = Picamera2()
        # Cache the mode table now: reading `.sensor_modes` reconfigures the camera to
        # probe each raw mode, so touching it after configure()/start() drops us onto
        # whatever it probed last (the 14 fps full-res mode).
        self._modes = list(self._picam.sensor_modes)

        # picamera2 otherwise pins FrameDuration to 30 fps regardless of the sensor mode,
        # so a fixed limit is always set: from `framerate` if given, else from the
        # selected sensor mode's own max rate (~56 fps for 2304x1296).
        if framerate > 0.0:
            fd = int(round(1_000_000.0 / framerate))
        else:
            mode_fps = self._mode_fps((sensor_w, sensor_h)) or 50.0
            fd = int(round(1_000_000.0 / mode_fps))
        ctrls = {"FrameDurationLimits": (fd, fd)}
        if af_mode == "manual":
            ctrls["AfMode"] = controls.AfModeEnum.Manual
            ctrls["LensPosition"] = lens_position
        else:
            ctrls["AfMode"] = controls.AfModeEnum.Continuous

        cfg = self._picam.create_video_configuration(
            main={"size": (self._width, self._height), "format": "RGB888"},
            sensor={"output_size": (sensor_w, sensor_h), "bit_depth": 10},
            transform=Transform(hflip=int(hflip), vflip=int(vflip)),
            controls=ctrls,
            buffer_count=6,
            queue=False,  # always hand back the newest frame, never a queued stale one
        )
        self._picam.configure(cfg)

        applied = self._picam.camera_configuration()
        got_w, got_h = applied["main"]["size"]
        if (got_w, got_h) != (self._width, self._height):
            self.get_logger().warn(
                "requested %dx%d but ISP produced %dx%d - update image_features' "
                "camera_width/height to match" % (self._width, self._height, got_w, got_h))
            self._width, self._height = got_w, got_h
        sensor_fps = self._mode_fps(applied["sensor"]["output_size"])
        if sensor_fps is not None and sensor_fps < 50.0:
            self.get_logger().warn(
                "sensor mode %s runs at %.1f fps - BELOW the 50 Hz image_features loop"
                % (applied["sensor"]["output_size"], sensor_fps))

        # -- publishers --------------------------------------------------------------
        self._pub = self.create_publisher(Image, image_topic, _image_qos(image_qos))
        self._info_pub = None
        self._info_msg = None
        if publish_camera_info:
            if camera_info_file:
                try:
                    self._info_msg = _load_camera_info(camera_info_file, self._frame_id)
                    self._info_pub = self.create_publisher(
                        CameraInfo, camera_info_topic, _image_qos("reliable"))
                    self.get_logger().info("camera_info: %s" % camera_info_file)
                except Exception as exc:  # noqa: BLE001 - report and carry on
                    self.get_logger().warn(
                        "could not load camera_info_file '%s': %s"
                        % (camera_info_file, exc))
            else:
                self.get_logger().warn(
                    "publish_camera_info:=true but camera_info_file is empty - skipping")

        # -- worker threads --------------------------------------------------------------
        self._running = True
        self._frame = None            # (data_bytes, w, h, stamp), newest only
        self._frame_lock = threading.Lock()
        self._frame_evt = threading.Event()
        self._intervals = deque(maxlen=512)  # inter-frame gaps, seconds
        self._n_captured = 0
        self._n_published = 0
        self._n_dropped = 0

        self._picam.start()
        self.get_logger().info(
            "camera_node: %dx%d bgr8 on %s (%s), sensor %s @ %.1f fps, hflip=%d vflip=%d"
            % (self._width, self._height, image_topic, image_qos,
               applied["sensor"]["output_size"], sensor_fps or -1.0, hflip, vflip))

        self._cap_thread = threading.Thread(
            target=self._capture_loop, name="cam_capture", daemon=True)
        self._pub_thread = threading.Thread(
            target=self._publish_loop, name="cam_publish", daemon=True)
        self._cap_thread.start()
        self._pub_thread.start()
        self._stats_timer = self.create_timer(5.0, self._log_stats)

    # -- helpers ---------------------------------------------------------------------

    def _mode_fps(self, size):
        for m in self._modes:
            if tuple(m["size"]) == tuple(size):
                return float(m["fps"])
        return None

    # -- capture -------------------------------------------------------------------------

    def _capture_loop(self):
        failures = 0
        last_restart = None
        last = None
        while self._running and rclpy.ok():
            try:
                req = self._picam.capture_request(wait=self._watchdog_timeout_s)
            except TimeoutError:
                failures += 1
                self.get_logger().warn(
                    "capture timeout (%d/%d)" % (failures, self._max_failures))
                if failures >= self._max_failures:
                    self.get_logger().fatal(
                        "camera delivered no frames after %d timeouts - shutting down"
                        % failures)
                    self._running = False
                    rclpy.try_shutdown()
                    return
                if failures % self._failures_before_restart == 0:
                    now = time.monotonic()
                    since = None if last_restart is None else now - last_restart
                    if since is None or since >= self._restart_cooldown_s:
                        self._restart_camera()
                        last_restart = now
                    else:
                        self.get_logger().warn(
                            "restart skipped, %.1fs of %.0fs cooldown remaining "
                            "(likely CPU contention, not a wedged camera)"
                            % (self._restart_cooldown_s - since, self._restart_cooldown_s))
                continue
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error("capture error: %s" % exc)
                time.sleep(0.1)
                continue

            failures = 0
            try:
                arr = req.make_array("main")            # HxWx3 uint8, BGR, packed
            finally:
                req.release()
            stamp = self.get_clock().now().to_msg()

            now = time.monotonic()
            if last is not None:
                self._intervals.append(now - last)
            last = now
            self._n_captured += 1

            with self._frame_lock:
                if self._frame is not None:
                    self._n_dropped += 1
                self._frame = (arr.tobytes(), arr.shape[1], arr.shape[0], stamp)
            self._frame_evt.set()

    def _restart_camera(self):
        self.get_logger().warn("attempting camera stop/start recovery")
        try:
            self._picam.stop()
            time.sleep(0.3)
            self._picam.start()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("camera recovery failed: %s" % exc)

    # -- publish -----------------------------------------------------------------------

    def _publish_loop(self):
        while self._running and rclpy.ok():
            if not self._frame_evt.wait(timeout=0.5):
                continue
            self._frame_evt.clear()
            with self._frame_lock:
                frame = self._frame
                self._frame = None
            if frame is None:
                continue
            data, w, h, stamp = frame

            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            msg.height = h
            msg.width = w
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = w * 3
            msg.data = array.array("B", data)  # rclpy buffer fast-path
            self._pub.publish(msg)
            self._n_published += 1

            if self._info_pub is not None:
                self._info_msg.header.stamp = stamp
                self._info_pub.publish(self._info_msg)

    # -- stats -----------------------------------------------------------------------

    def _log_stats(self):
        iv = list(self._intervals)
        if iv:
            iv_sorted = sorted(iv)
            mean_hz = len(iv) / sum(iv)
            p95_ms = iv_sorted[min(len(iv_sorted) - 1, int(0.95 * len(iv_sorted)))] * 1e3
        else:
            mean_hz = 0.0
            p95_ms = 0.0
        self.get_logger().info(
            "camera: %.1f fps  p95 gap %.1f ms  published %d  dropped %d"
            % (mean_hz, p95_ms, self._n_published, self._n_dropped))
        self._n_published = 0
        self._n_dropped = 0

    # -- shutdown --------------------------------------------------------------------

    def destroy_node(self):
        self._running = False
        self._frame_evt.set()
        for t in (getattr(self, "_cap_thread", None), getattr(self, "_pub_thread", None)):
            if t is not None:
                t.join(timeout=1.5)
        try:
            self._picam.stop()
            self._picam.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def _load_camera_info(path: str, frame_id: str) -> CameraInfo:
    """Parse an OpenCV / ROS `camera_info` YAML into a CameraInfo message.

    Mirrors TE3002B_mcrChallenge/pzb_camera camera_raw_publisher._load_camera_info.
    """
    import yaml

    with open(path) as fh:
        d = yaml.safe_load(fh)
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = int(d["image_width"])
    msg.height = int(d["image_height"])
    msg.distortion_model = d.get("distortion_model", "plumb_bob")
    msg.d = list(map(float, d["distortion_coefficients"]["data"]))
    msg.k = list(map(float, d["camera_matrix"]["data"]))
    msg.r = list(map(float, d.get("rectification_matrix", {"data": [1, 0, 0, 0, 1, 0, 0, 0, 1]})["data"]))
    msg.p = list(map(float, d["projection_matrix"]["data"]))
    return msg


def main(args=None):
    # Fast DDS's default shared-memory segment (512 KiB) is smaller than one published
    # frame (2.24 MB at 1152x648 bgr8) - under CPU contention the oversized message can't
    # keep up with the small segment and gets silently dropped. camera.launch.py sets
    # this via additional_env; setdefault here covers a bare `ros2 run`, and respects an
    # explicit override. Must be set before rclpy.init() creates the DDS participant.
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
