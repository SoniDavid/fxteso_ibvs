"""Run image_features' ArUco detection on the live camera and time it.

    ros2 run quad_cam probe_detect
    ros2 run quad_cam probe_detect --ros-args -p marker_dict:=7x7 -p image_qos:=sensor_data

Reports cv2.aruco.detectMarkers cost (p50/p95/p99), which IDs are seen, and the fraction
of frames that would give image_features a valid lock (exactly the IDs {4,6,8,10}). This
is the `first-flight-checklist.md` go/no-go number: p99 must sit under the 20 ms loop
budget at the resolution you actually publish.
"""

import os
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image

import cv2

_NUM = ParameterDescriptor(dynamic_typing=True)   # accept int or float from the CLI
_REQUIRED_IDS = {4, 6, 8, 10}
_DICTS = {
    "4x4": cv2.aruco.DICT_4X4_50, "5x5": cv2.aruco.DICT_5X5_50,
    "6x6": cv2.aruco.DICT_6X6_50, "7x7": cv2.aruco.DICT_7X7_50,
}


def _get_dictionary(key):
    enum = _DICTS[key]
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(enum)
    return cv2.aruco.Dictionary_get(enum)  # older cv2


def _detect(frame, dictionary, params):
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = det.detectMarkers(frame)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=params)
    return corners, ids


def _make_params():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


class ProbeDetect(Node):

    def __init__(self):
        super().__init__("probe_detect")
        topic = self.declare_parameter("image_topic", "/quad/camera/image_raw").value
        qos_kind = self.declare_parameter("image_qos", "reliable").value
        dict_key = self.declare_parameter("marker_dict", "7x7").value
        self._report_every = float(self.declare_parameter("report_period_s", 5.0).value)

        # Mirrors image_features.cpp's aruco3 tuning so this probe measures the same
        # configuration that will actually run - default computed the same way
        # hardware.launch.py computes it (zD=1.2, target_scale=0.5, module3wide_2304,
        # margin=0.7): recompute if that deployment geometry changes.
        use_aruco3 = bool(self.declare_parameter("use_aruco3_detection", True).value)
        min_marker_ratio = float(
            self.declare_parameter("min_marker_length_ratio", 0.0289, _NUM).value)

        self._dictionary = _get_dictionary(dict_key)
        self._params = _make_params()
        self._params.useAruco3Detection = use_aruco3
        self._params.minMarkerLengthRatioOriginalImg = min_marker_ratio

        reliability = (QoSReliabilityPolicy.BEST_EFFORT if qos_kind == "sensor_data"
                       else QoSReliabilityPolicy.RELIABLE)
        qos = QoSProfile(reliability=reliability, history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1, durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(Image, topic, self._cb, qos)

        self._ms = []
        self._n = 0
        self._n_lock = 0
        self._ids_seen = {}
        self.create_timer(self._report_every, self._report)
        self.get_logger().info(
            "probing %s, DICT_%s_50, aruco3=%s%s"
            % (topic, dict_key.upper(), use_aruco3,
               " ratio=%.4f" % min_marker_ratio if use_aruco3 else ""))

    def _cb(self, msg: Image):
        if msg.encoding not in ("bgr8", "rgb8", "mono8"):
            self.get_logger().warn("unexpected encoding %s" % msg.encoding)
            return
        ch = 1 if msg.encoding == "mono8" else 3
        frame = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, ch)

        t0 = time.perf_counter()
        _, ids = _detect(frame, self._dictionary, self._params)
        self._ms.append((time.perf_counter() - t0) * 1e3)
        self._n += 1

        flat = set(int(i) for i in ids.flatten()) if ids is not None else set()
        for i in flat:
            self._ids_seen[i] = self._ids_seen.get(i, 0) + 1
        if flat == _REQUIRED_IDS:
            self._n_lock += 1

    def _report(self):
        if not self._ms:
            self.get_logger().warn("no frames received")
            return
        xs = sorted(self._ms)

        def pct(p):
            return xs[min(len(xs) - 1, int(p * len(xs)))]

        self.get_logger().info(
            "detectMarkers ms  p50 %.1f  p95 %.1f  p99 %.1f  max %.1f | "
            "valid lock %d/%d (%.0f%%) | ids seen %s"
            % (pct(0.50), pct(0.95), pct(0.99), xs[-1],
               self._n_lock, self._n, 100.0 * self._n_lock / self._n,
               dict(sorted(self._ids_seen.items()))))
        self._ms.clear()
        self._n = 0
        self._n_lock = 0
        self._ids_seen.clear()


def main(args=None):
    # See camera_node.py's main() - the image topic's messages are far bigger than Fast
    # DDS's default shared-memory segment. Must be set before rclpy.init().
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS', 'LARGE_DATA')
    rclpy.init(args=args)
    node = ProbeDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
