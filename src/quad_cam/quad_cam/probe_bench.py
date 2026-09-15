"""Live benchmark sampler for the IBVS stack on the Pi.

Run it alongside `ros2 launch quad_cam bench.launch.py`. Every report_period_s it
prints, and on exit it summarises:

  * achieved loop rate for each control/estimation node (from its always-on output topic)
  * per-process CPU%, system per-core CPU%, load average, free RAM
  * Pi SoC temperature, throttle/undervoltage flags, current CPU clock

The per-loop pass criterion is `achieved >= 0.98 * target` with few overruns; cross-check
against each node's own `loop:` line (SimRate self-report) in the launch output.

  ros2 run quad_cam probe_bench
  ros2 run quad_cam probe_bench --ros-args -p duration_s:=120
"""

import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Bool

_NUM = ParameterDescriptor(dynamic_typing=True)   # accept int or float from the CLI

try:
    import psutil
except ImportError:  # pragma: no cover
    psutil = None

# node label -> (topic, msg type, target Hz). The image_features topic is a parameter
# (`imfeat_topic`): bench.launch.py load:=synthetic remaps its real output to
# /ImFeat_valid_probe (the default here) while bench_feeder owns /ImFeat_valid; for
# load:=plate|none pass -p imfeat_topic:=/ImFeat_valid. fixed_eso/pos_ctrl publish
# nothing until ImFeat_valid is true.
_LOOPS = [
    ('td_attitude',         '/attitude_td_error',       Vector3,   100.0),
    ('td_linear',           '/position_td_error',       Vector3,   100.0),
    ('td_attitude_desired', '/attitude_des_td_error',   Vector3,    50.0),
    ('fixed_eso',           '/ImFeat_estimates_fxt',    Quaternion, 50.0),
    ('pos_ctrl',            '/ibvs_control_input',      Quaternion, 50.0),
    ('att_ctrl',            '/quad_torques',            Vector3,   100.0),
]

_PROCS = ('image_features', 'fixed_eso', 'pos_ctrl', 'td_attitude', 'td_attitude_desired',
          'td_linear', 'att_ctrl', 'camera_node', 'bench_feeder')

# bits 0-3 = happening now, bits 16-19 = happened since boot
_THROTTLE_NOW = {0: 'under-voltage', 1: 'arm-freq-capped', 2: 'throttled',
                 3: 'soft-temp-limit'}
_THROTTLE_EVER = {16: 'under-voltage', 17: 'arm-freq-cap', 18: 'throttling',
                  19: 'soft-temp-limit'}


def _count_qos():
    # RELIABLE + a deep queue: the fxteso nodes all publish default (RELIABLE) QoS, and an
    # accurate count needs every message - a BEST_EFFORT depth-1 sub silently drops ~5% at
    # 100 Hz. The callback only increments an int, so there is no back-pressure.
    return QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=200,
                      durability=QoSDurabilityPolicy.VOLATILE)


def _vcgencmd(arg):
    try:
        return subprocess.run(['vcgencmd', arg], capture_output=True, text=True,
                              timeout=2.0).stdout.strip()
    except Exception:
        return ''


def _cpu_khz():
    try:
        with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq') as fh:
            return int(fh.read().strip())
    except Exception:
        return 0


class ProbeBench(Node):

    def __init__(self):
        super().__init__('probe_bench')
        self._period = float(self.declare_parameter('report_period_s', 5.0, _NUM).value)
        self._duration = float(self.declare_parameter('duration_s', 0.0, _NUM).value)
        # first N windows are startup (camera AE settle, node stagger) - shown but not
        # counted toward the worst-case summary.
        self._warmup = int(self.declare_parameter('warmup_reports', 1, _NUM).value)
        imfeat = str(self.declare_parameter(
            'imfeat_topic', '/ImFeat_valid_probe').value)

        # image_features' rate topic is a parameter (see _LOOPS comment); the rest are fixed.
        self._loops = [('image_features', imfeat, Bool, 50.0)] + list(_LOOPS)
        self._counts = {label: 0 for label, *_ in self._loops}
        for label, topic, msg_type, _hz in self._loops:
            self.create_subscription(
                msg_type, topic,
                lambda _m, ll=label: self._counts.__setitem__(ll, self._counts[ll] + 1),
                _count_qos())

        self._procs = {}          # pid -> psutil.Process
        self._t0 = time.monotonic()
        self._win_t0 = self._t0
        self._min_hz = {}         # label -> worst achieved Hz seen (while active)
        self._peak_cpu = 0.0
        self._peak_temp = 0.0
        self._throttle_seen = set()
        self._n_reports = 0

        if psutil is None:
            self.get_logger().warn('psutil not importable - CPU/RAM columns disabled')
        self.get_logger().info(
            'probe_bench: reporting every %.0f s%s'
            % (self._period, '' if self._duration <= 0 else
               ', stopping after %.0f s' % self._duration))
        self.create_timer(self._period, self._report)

    # -- sampling ---------------------------------------------------------------------

    def _match(self, p):
        """Return the _PROCS label for process p, or None. Exact basename match, longest
        target first so 'td_attitude_desired' is not shadowed by 'td_attitude'."""
        try:
            cand = {p.info['name'] or ''}
            for c in (p.info['cmdline'] or [])[:1]:
                cand.add(os.path.basename(c))
        except Exception:
            return None
        for name in sorted(_PROCS, key=len, reverse=True):
            if name in cand:
                return name
        return None

    def _proc_cpu(self):
        if psutil is None:
            return None
        seen = {}
        for p in psutil.process_iter(['pid', 'name', 'cmdline']):
            name = self._match(p)
            if name is None:
                continue
            pr = self._procs.get(p.info['pid'])
            if pr is None:
                try:
                    pr = psutil.Process(p.info['pid'])
                    pr.cpu_percent(None)   # prime
                except Exception:
                    continue
                self._procs[p.info['pid']] = pr
            try:
                seen[name] = seen.get(name, 0.0) + pr.cpu_percent(None)
            except Exception:
                pass
        return seen

    def _report(self):
        now = time.monotonic()
        dt = now - self._win_t0
        self._win_t0 = now
        self._n_reports += 1
        elapsed = now - self._t0

        counted = self._n_reports > self._warmup
        lines = ['', '=== probe_bench  t+%.0fs%s ==='
                 % (elapsed, '' if counted else '  (warm-up, not scored)')]
        lines.append('  loop                    target   achieved')
        for label, _topic, _mt, hz in self._loops:
            r = self._counts[label] / dt if dt > 0 else 0.0
            self._counts[label] = 0
            if counted and r > 1.0:   # "active" - skip not-yet-locked / disabled loops
                self._min_hz[label] = min(self._min_hz.get(label, r), r)
            flag = ''
            if r <= 1.0:
                flag = '  (idle/off)'
            elif r < 0.98 * hz:
                flag = '  <-- BELOW'
            lines.append('  %-22s  %5.0f    %6.1f%s' % (label, hz, r, flag))

        cpu = self._proc_cpu()
        if cpu is not None:
            total = sum(cpu.values())
            if counted:
                self._peak_cpu = max(self._peak_cpu, total)
            per_proc = '  '.join('%s=%.0f' % (k, v) for k, v in sorted(cpu.items()))
            lines.append('  cpu% per proc: ' + (per_proc or '(no matched procs yet)'))
            per_core = psutil.cpu_percent(percpu=True)
            vm = psutil.virtual_memory()
            lines.append('  system: cores ' + ' '.join('%2.0f' % c for c in per_core)
                         + ' | load %.2f %.2f %.2f' % os.getloadavg()
                         + ' | RAM free %.0f MB' % (vm.available / 1e6)
                         + ' | matched-proc total %.0f%%' % total)

        temp_s = _vcgencmd('measure_temp')          # temp=54.0'C
        thr_s = _vcgencmd('get_throttled')          # throttled=0x0
        temp = 0.0
        try:
            temp = float(temp_s.split('=')[1].rstrip("'C"))
            if counted:
                self._peak_temp = max(self._peak_temp, temp)
        except Exception:
            pass
        now_flags, ever_flags = [], []
        try:
            bits = int(thr_s.split('=')[1], 16)
            for b, name in _THROTTLE_NOW.items():
                if bits & (1 << b):
                    now_flags.append(name)
                    if counted:
                        self._throttle_seen.add(name)
            for b, name in _THROTTLE_EVER.items():
                if bits & (1 << b):
                    ever_flags.append(name)
        except Exception:
            pass
        lines.append('  soc: %.1f C | clock %.0f MHz | throttled NOW: %s | since boot: %s'
                     % (temp, _cpu_khz() / 1000.0,
                        ', '.join(now_flags) or 'no',
                        ', '.join(ever_flags) or 'no'))
        self.get_logger().info('\n'.join(lines))

        if self._duration > 0 and elapsed >= self._duration:
            self._summary()
            rclpy.shutdown()

    def _summary(self):
        if getattr(self, '_summarised', False):
            return
        self._summarised = True
        lines = ['', '=== probe_bench SUMMARY (%.0f s, %d reports) ==='
                 % (time.monotonic() - self._t0, self._n_reports),
                 '  worst achieved rate per active loop:']
        for label, _topic, _mt, hz in self._loops:
            if label in self._min_hz:
                m = self._min_hz[label]
                verdict = 'OK' if m >= 0.98 * hz else 'FAIL'
                lines.append('    %-22s  %6.1f / %.0f Hz   %s' % (label, m, hz, verdict))
        lines.append('  peak matched-proc CPU: %.0f%%' % self._peak_cpu)
        lines.append('  peak SoC temp: %.1f C' % self._peak_temp)
        lines.append('  throttle flags seen: %s'
                     % (', '.join(sorted(self._throttle_seen)) or 'none'))
        self.get_logger().info('\n'.join(lines))

    def destroy_node(self):
        try:
            self._summary()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    # Transport comes from the caller; bench.sh exports LARGE_DATA to match bench.launch.py.
    rclpy.init(args=args)
    node = ProbeBench()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
