#!/usr/bin/env python3
"""Bench test a Raspberry Pi Camera Module 3 Wide on a Pi 5 against this repo's camera presets.

    python3 camera_module3_wide_test.py                    # the full suite
    python3 camera_module3_wide_test.py --preview          # ... with a live window
    python3 camera_module3_wide_test.py --only rate --frames 1000
    python3 camera_module3_wide_test.py --skip video,still
    python3 camera_module3_wide_test.py --calib 20         # checkerboard captures, no tests

    python3 camera_module3_wide_test.py --doctor            # environment preflight only

Standalone on purpose - scp it to the Pi and run it. Nothing here imports ROS or this
workspace, and the only non-stdlib dependency is picamera2.

On Raspberry Pi OS picamera2 is preinstalled and the tests just run. On Ubuntu 24.04 they
cannot: noble ships upstream libcamera 0.2.0, whose libcamera-ipa package contains
ipa_rpi_vc4.so and no ipa_rpi_pisp.so, and the Pi 5's ISP is PiSP. There is no
python3-picamera2 and no rpicam-apps in the archive either. So on Ubuntu the camera stack
has to be built from Raspberry Pi's libcamera fork first - run --doctor, which reports
exactly which link of that chain is missing, and tools/setup_pi5_ubuntu_camera.sh, which
builds it.

The thresholds are not invented. They are src/quad_description/config/cameras.yaml's
module3wide_2304 preset, which quad_px4/launch/hardware.launch.py flies by default. That
preset exists because 2304x1296 is the widest Module 3 mode that clears the 50 Hz
image_features/fixed_eso loop - and if the hardware misses it, the loop just repeats frames
and nothing anywhere reports that. Gazebo renders at a free 50 fps, so the sim cannot catch
it either. This script is the only place the real number gets checked.

Exit status is 0 only if every check passed.
"""
import argparse
import glob
import json
import math
import os
import platform
import shutil
import subprocess
import sys
import time

from datetime import datetime

# Imported softly on purpose. --doctor's whole job is to explain a machine where this
# import fails, so it must not be the thing that kills the process.
PICAMERA2_ERROR = None
try:
    from picamera2 import Picamera2, Preview
    from picamera2.encoders import H264Encoder
    from picamera2.outputs import FfmpegOutput, FileOutput
    from libcamera import controls as libcontrols
except ImportError as exc:
    PICAMERA2_ERROR = exc
    Picamera2 = Preview = H264Encoder = FfmpegOutput = FileOutput = libcontrols = None


# --- the spec, mirrored from src/quad_description/config/cameras.yaml ------------------------
# Kept as literals rather than read from the YAML so the file stays scp-able on its own.
# If cameras.yaml changes, change these too - they are the whole point of the script.

LOOP_HZ = 50.0            # image_features and fixed_eso both run here (camera_presets.py:35)
MODEL = 'imx708_wide'     # what separates the Wide from the standard Module 3 (imx708)
ASSUMED_DISTORTION = [-0.30, 0.10, 0.0, 0.0, -0.02]     # cameras.yaml:73, not a measurement

PRIMARY = {'preset': 'module3wide_2304', 'size': (2304, 1296), 'fps': 56.03,
           'hfov_deg': 102.0, 'crop': 1.0}
FALLBACK = {'preset': 'module3wide_1536', 'size': (1536, 864), 'fps': 120.13,
            'hfov_deg': 78.9, 'crop': 0.6667}

FPS_TOLERANCE = 0.02      # advertised mode rate vs the preset, fractional
CROP_TOLERANCE = 0.02     # ScalerCrop vs the full active area, fractional

TESTS = ('detect', 'modes', 'fov', 'rate', 'still', 'focus', 'video', 'metadata')


# --- reporting --------------------------------------------------------------------------

class Report:
    """Every assertion, printed as it happens and again as a table at the end.

    A check prints what was measured next to what was required and where the requirement
    came from, because a bare FAIL on a bench test tells you nothing you can act on.
    """

    def __init__(self):
        self.checks = []
        self.data = {}
        self.skipped = []

    def check(self, label, ok, measured, expected, source=''):
        self.checks.append({'label': label, 'ok': bool(ok), 'measured': str(measured),
                            'expected': str(expected), 'source': source})
        print('    %-34s %-22s (need %s)  %s'
              % (label, measured, expected, 'PASS' if ok else 'FAIL'))
        return bool(ok)

    def note(self, text):
        print('    %s' % text)

    def failed(self):
        return [c for c in self.checks if not c['ok']]

    def table(self):
        print('')
        print('=' * 92)
        width = max([len(c['label']) for c in self.checks] + [10])
        for c in self.checks:
            # Truncate rather than let one long value knock every later column out of line.
            print('%-4s %-*s  %-22s  need %-20s %s'
                  % ('PASS' if c['ok'] else 'FAIL', width, c['label'],
                     clip(c['measured'], 22), clip(c['expected'], 20), c['source']))
        bad = self.failed()
        print('=' * 92)
        print('%d checks, %d failed%s'
              % (len(self.checks), len(bad),
                 (', skipped: %s' % ', '.join(self.skipped)) if self.skipped else ''))


def clip(text, width):
    return text if len(text) <= width else text[:width - 1] + '~'


def pct(values, q):
    """Nearest-rank percentile. statistics.quantiles needs >1 point and we may have few."""
    if not values:
        return float('nan')
    ordered = sorted(values)
    i = int(round((len(ordered) - 1) * q))
    return ordered[max(0, min(len(ordered) - 1, i))]


def mean(values):
    return sum(values) / len(values) if values else float('nan')


def fx_of(width, hfov_deg):
    """The focal length in pixels image_features will be handed - camera_presets.py:70."""
    return width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))


# --- camera helpers ---------------------------------------------------------------------

def find_mode(modes, size):
    """The sensor mode whose readout is exactly `size`, preferring the deepest bit depth."""
    matches = [m for m in modes if tuple(m['size']) == tuple(size)]
    if not matches:
        return None
    return sorted(matches, key=lambda m: m.get('bit_depth', 0))[-1]


def video_config(cam, spec, mode):
    """A video configuration with the SENSOR MODE pinned, not merely the output size.

    Asking for a 2304x1296 main stream is not the same as asking the sensor for its
    2304x1296 readout: libcamera is free to serve it by scaling a different mode, which
    changes both the field of view and the achievable rate. The `sensor=` argument is the
    explicit way to say which readout, and predates nothing older than picamera2 0.3.18 -
    hence the raw= fallback.

    FrameRate collapses FrameDurationLimits to a fixed budget, which also caps how far
    auto-exposure may stretch a frame. Without it a dim room silently halves the rate.
    """
    main = {'size': tuple(spec['size']), 'format': 'RGB888'}
    ctrls = {'FrameRate': spec['fps']}
    try:
        return cam.create_video_configuration(
            main=main, controls=ctrls, buffer_count=6,
            sensor={'output_size': tuple(mode['size']), 'bit_depth': mode['bit_depth']})
    except (TypeError, KeyError):
        return cam.create_video_configuration(
            main=main, controls=ctrls, buffer_count=6,
            raw={'size': tuple(mode['size']), 'format': mode.get('unpacked', mode['format'])})


def restart(cam, config):
    """Reconfigure between subtests. Picamera2 refuses to configure while running."""
    try:
        if cam.started:
            cam.stop()
    except Exception:
        pass
    cam.configure(config)
    cam.start()


def start_preview(cam):
    """QTGL on a desktop session, DRM on the bare console, nothing over plain ssh.

    A preview must never be the reason the suite dies - it is an operator convenience, and
    the same script has to run headless from an ssh checklist.
    """
    if os.environ.get('WAYLAND_DISPLAY') or os.environ.get('DISPLAY'):
        order = [('QTGL', Preview.QTGL), ('QT', Preview.QT)]
    else:
        order = [('DRM', Preview.DRM)]
    for name, kind in order:
        try:
            cam.start_preview(kind)
            print('preview: %s' % name)
            return name
        except Exception as exc:
            print('preview: %s unavailable (%s)' % (name, exc))
    print('preview: none - running headless')
    return None


def vcgencmd(arg):
    try:
        out = subprocess.run(['vcgencmd'] + arg.split(), capture_output=True, text=True,
                             timeout=5)
        return out.stdout.strip() or out.stderr.strip()
    except Exception as exc:
        return 'unavailable (%s)' % exc


# --- environment preflight ---------------------------------------------------------------
# Everything here is read-only and works with picamera2 absent, because on Ubuntu that is
# the normal starting state and "ImportError" on its own tells you nothing actionable.

def _read(path):
    try:
        with open(path, 'rb') as fh:
            return fh.read().decode('utf-8', 'replace').replace('\x00', '').strip()
    except Exception:
        return ''


def _run(cmd):
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        return (out.stdout + out.stderr).strip()
    except Exception as exc:
        return 'unavailable (%s)' % exc


def _ipa_hits(pattern):
    """IPA modules matching `pattern`, wherever this machine keeps them.

    The nesting is not stable across versions: libcamera 0.7 installs to
    <libdir>/libcamera/ipa/, while Ubuntu's packaged 0.2 puts them directly in
    <libdir>/libcamera/. Searching only one level reports a perfectly good build as broken,
    which is exactly what this function got wrong before.
    """
    bases = []
    for root in ('/usr/lib', '/usr/local/lib'):
        bases += glob.glob(os.path.join(root, '*', 'libcamera'))
        bases += glob.glob(os.path.join(root, 'libcamera'))
    env = os.environ.get('LIBCAMERA_IPA_MODULE_PATH')
    if env:
        bases += [d for d in env.split(':') if d]
    hits = []
    for base in bases:
        if os.path.isdir(base):
            hits += glob.glob(os.path.join(base, '**', pattern), recursive=True)
    return sorted(set(hits))


# The only directories meson or apt ever install the bindings into. Enumerated rather than
# walked: a recursive glob over /usr/lib takes minutes, and --doctor is the first thing anyone
# runs on a broken machine.
_BINDING_GLOBS = (
    '/usr/local/lib/python3/dist-packages/libcamera/_libcamera*.so',
    '/usr/local/lib/python3.*/dist-packages/libcamera/_libcamera*.so',
    '/usr/local/lib/python3.*/site-packages/libcamera/_libcamera*.so',
    '/usr/lib/python3/dist-packages/libcamera/_libcamera*.so',
    '/usr/lib/python3.*/dist-packages/libcamera/_libcamera*.so',
)


def _find_bindings():
    """The installed libcamera Python package on disk, importable or not."""
    hits = []
    for pattern in _BINDING_GLOBS:
        hits += glob.glob(pattern)
    return sorted(set(hits))


def doctor():
    """Report which link of the camera chain is missing, in the order it breaks.

    Returns the number of hard failures. Written for Ubuntu, harmless on Pi OS - on Pi OS
    every line simply says OK.
    """
    hard = 0

    def line(ok, label, detail, fix='', fatal=True):
        nonlocal hard
        mark = 'OK  ' if ok else ('FAIL' if fatal else 'WARN')
        print('%s %-26s %s' % (mark, label, detail))
        if not ok:
            if fatal:
                hard += 1
            if fix:
                for part in fix.split('\n'):
                    print('     -> %s' % part)

    print('--- environment preflight %s' % ('-' * 47))

    model = _read('/proc/device-tree/model') or 'unknown'
    is_pi5 = 'Raspberry Pi 5' in model
    line(is_pi5, 'board', model,
         'This script targets the Pi 5. On a Pi 4 or earlier the ISP is VC4, not PiSP, and\n'
         'the libcamera notes below do not apply - but 2304x1296@56fps will not be reachable\n'
         'either, which is the whole question cameras.yaml asks.', fatal=False)

    pretty = ''
    for row in _read('/etc/os-release').splitlines():
        if row.startswith('PRETTY_NAME='):
            pretty = row.split('=', 1)[1].strip('"')
    is_ubuntu = 'Ubuntu' in pretty
    line(True, 'distribution', '%s (%s)' % (pretty or 'unknown', platform.machine()))

    kernel = platform.release()
    # Ubuntu's generic kernel has no Pi sensor drivers at all; linux-raspi does.
    ok_kernel = (not is_ubuntu) or '-raspi' in kernel
    line(ok_kernel, 'kernel', kernel,
         'Ubuntu needs the Raspberry Pi kernel flavour for the IMX708 and PiSP drivers:\n'
         '    sudo apt install linux-raspi   # then reboot')

    cfg_path = '/boot/firmware/config.txt'
    cfg = _read(cfg_path)
    has_cam = ('camera_auto_detect=1' in cfg) or ('dtoverlay=imx708' in cfg)
    line(bool(cfg) and has_cam, 'config.txt', cfg_path if cfg else 'not found',
         'Add ONE of these to %s and reboot:\n'
         '    camera_auto_detect=1\n'
         '    dtoverlay=imx708,cam0     # explicit; use cam1 for the other connector' % cfg_path)

    bound = glob.glob('/sys/bus/i2c/drivers/imx708/*-001a') + \
        glob.glob('/sys/bus/i2c/drivers/imx708/*')
    bound = [b for b in bound if os.path.basename(b) not in ('bind', 'unbind', 'uevent',
                                                             'module')]
    line(bool(bound), 'imx708 driver bound', ', '.join(bound) or 'no i2c device bound',
         'The kernel cannot see the sensor. Check the ribbon seating and orientation, then\n'
         '    dmesg | grep -i imx708\n'
         'A cable in backwards is the single most common cause and reports exactly this.')

    media = sorted(glob.glob('/dev/media*'))
    video = sorted(glob.glob('/dev/video*'))
    line(bool(media), 'media devices', ', '.join(media) or 'none',
         'No /dev/media* means the CFE/PiSP drivers did not load.')
    line(True, 'video devices', ', '.join(video) or 'none', fatal=False)

    # libpisp is checked BEFORE the IPA because it is the usual reason the IPA is absent:
    # libcamera resolves it through pkg-config and, when it is missing, quietly configures
    # the pisp pipeline out instead of failing the build.
    pisp_lib = (glob.glob('/usr/local/lib/*/libpisp.so*') + glob.glob('/usr/lib/*/libpisp.so*')
                + glob.glob('/usr/local/lib/libpisp.so*'))
    line(bool(pisp_lib), 'libpisp', pisp_lib[0] if pisp_lib else 'not installed',
         'Not packaged for Ubuntu, and libcamera\'s rpi/pisp pipeline needs its headers.\n'
         'It must be built and installed BEFORE libcamera, or libcamera silently drops\n'
         'pisp support:  bash tools/setup_pi5_ubuntu_camera.sh')

    # The decisive one on Ubuntu. noble's libcamera-ipa ships ipa_rpi_vc4.so only.
    pisp = _ipa_hits('ipa_rpi_pisp.so')
    vc4 = _ipa_hits('ipa_rpi_vc4.so')
    dirs = sorted(set(os.path.dirname(h) for h in _ipa_hits('*.so')))
    line(bool(pisp), 'PiSP IPA module', pisp[0] if pisp else
         'absent (searched %s)' % (', '.join(dirs) or 'nothing'),
         'Build Raspberry Pi\'s fork, libpisp first:\n'
         '    bash tools/setup_pi5_ubuntu_camera.sh\n'
         'If libcamera IS already built, it was configured without pisp. Check with:\n'
         '    grep -i libpisp ~/src/libcamera/build/meson-logs/meson-log.txt')
    if not pisp:
        # Say what IS there. "absent" alone cannot distinguish a missing build from a
        # build that went somewhere this script did not look.
        for d in dirs:
            names = sorted(os.path.basename(f) for f in glob.glob(os.path.join(d, '*.so')))
            print('     -> %s holds: %s' % (d, ', '.join(names) or '(no .so files)'))
        if vc4:
            print('     -> %s is the Pi 4 ISP, not the Pi 5\'s' % vc4[0])

    tuning = []
    for root in ('/usr/share', '/usr/local/share'):
        tuning += glob.glob(os.path.join(root, 'libcamera', 'ipa', 'rpi', 'pisp',
                                         'imx708*.json'))
    # imx708_wide_noir.json is the no-IR-filter variant and is NOT this module's tuning;
    # matching it loosely would report OK for the wrong lens calibration.
    wide = [t for t in tuning if os.path.basename(t) == 'imx708_wide.json']
    others = sorted(os.path.basename(t) for t in tuning
                    if os.path.basename(t) != 'imx708_wide.json')
    line(bool(wide), 'imx708_wide tuning',
         wide[0] if wide else ('absent; found %s' % (', '.join(others) or 'nothing')),
         'Without imx708_wide.json libcamera falls back to another tuning: the images work,\n'
         'the colour and lens-shading correction are wrong for this lens.', fatal=False)

    libs = _run(['ldconfig', '-p'])
    picked = [l.strip() for l in libs.splitlines() if 'libcamera.so' in l]
    line(bool(picked), 'libcamera .so', picked[0] if picked else 'not in the linker cache',
         'After a source install run: sudo ldconfig')
    if len(picked) > 1:
        print('     -> %d libcamera.so entries; a source build in /usr/local must win over'
              ' the apt one' % len(picked))

    try:
        import libcamera as _lc
        line(True, 'python libcamera', getattr(_lc, '__file__', 'imported'))
    except ImportError as exc:
        found = _find_bindings()
        if found:
            # Installed but unreachable. meson's prefix-relative install directory is often
            # /usr/local/lib/python3/dist-packages - note "python3", no minor version -
            # and that exact path is on nobody's sys.path.
            pkg = os.path.dirname(os.path.dirname(found[0]))
            import site
            targets = [d for d in (site.getsitepackages() or []) if 'dist-packages' in d]
            target = targets[0] if targets else '/usr/local/lib/python3/dist-packages'
            line(False, 'python libcamera',
                 'built but not on sys.path (%s)' % exc,
                 'The bindings are installed at\n'
                 '    %s\n'
                 'which is not a directory Python searches. Bridge it once:\n'
                 '    echo %s | sudo tee %s/libcamera-local.pth\n'
                 'No rebuild needed - the compiled module is already correct.'
                 % (pkg, pkg, target))
        else:
            line(False, 'python libcamera', 'not importable (%s)' % exc,
                 'No _libcamera*.so anywhere under /usr/lib or /usr/local/lib, so the\n'
                 'bindings were never built. libcamera needs -Dpycamera=enabled and\n'
                 'pybind11-dev present at configure time.')

    # Checked before picamera2 because it is a cause, not a symptom: picamera2's
    # previews/__init__.py does an unconditional `from .drm_preview import DrmPreview`, and
    # that imports pykms. Without it `import picamera2` fails even headless with no DISPLAY,
    # and the resulting "No module named 'pykms'" looks like a preview problem when it is not.
    import importlib.util as _ilu
    has_pykms = _ilu.find_spec('pykms') is not None
    line(has_pykms, 'pykms (kmsxx)', 'importable' if has_pykms else 'not importable',
         'picamera2 cannot be imported at all without it. Pi OS ships python3-kms++;\n'
         'Ubuntu has no equivalent under any name, so it is a source build:\n'
         '    bash tools/setup_pi5_ubuntu_camera.sh --skip-libcamera')

    if PICAMERA2_ERROR is None:
        import picamera2 as _p2
        line(True, 'picamera2', getattr(_p2, '__version__', '') or _p2.__file__)
    else:
        line(False, 'picamera2', 'not importable (%s)' % PICAMERA2_ERROR,
             'Pi OS:   sudo apt install -y python3-picamera2\n'
             'Ubuntu:  no such package exists in noble, so pip is the only route:\n'
             '             sudo apt install -y libcap-dev python3-prctl python3-piexif\n'
             '             pip install --break-system-packages picamera2\n'
             '         libcap-dev is not optional - picamera2 pulls python-prctl, a C\n'
             '         extension whose build fails without those headers. And if the error\n'
             '         above says pykms, the pip install already worked: see that line.')

    # Advisory. start_preview() falls back to headless on its own, so this only sets
    # expectations for --preview rather than gating anything.
    # find_spec, not import: loading Qt costs a second and this only needs to know it exists.
    import importlib.util
    has_qt = importlib.util.find_spec('PyQt5') is not None
    kmspp = bool(glob.glob('/usr/lib/python3*/dist-packages/pykms*')
                 + glob.glob('/usr/local/lib/python3*/dist-packages/pykms*'))
    session = os.environ.get('WAYLAND_DISPLAY') or os.environ.get('DISPLAY') or 'none'
    line(has_qt, 'preview (--preview)',
         'PyQt5 %s, kms++ %s, session %s'
         % ('yes' if has_qt else 'no', 'yes' if kmspp else 'no', session),
         'The Qt preview needs:  sudo apt install -y python3-pyqt5 python3-opengl\n'
         'python3-kms++ is not packaged for Ubuntu, so the DRM (bare console) preview is\n'
         'unavailable here - run --preview from a desktop session, or leave it off.',
         fatal=False)

    hello = shutil.which('rpicam-hello') or shutil.which('libcamera-hello')
    line(bool(hello), 'rpicam-hello', hello or 'not installed',
         'Not required by this script, but it is the fastest way to prove the CSI link\n'
         'outside Python. Ubuntu has no rpicam-apps package; the setup script builds it.',
         fatal=False)

    try:
        import grp
        in_video = 'video' in [grp.getgrgid(g).gr_name for g in os.getgroups()]
    except Exception:
        in_video = True
    line(in_video, 'video group', 'member' if in_video else 'not a member',
         'sudo usermod -aG video $USER   # then log out and back in', fatal=False)

    print('--- preflight: %d blocking problem(s) %s' % (hard, '-' * 40))
    return hard


# --- subtests ---------------------------------------------------------------------------

def t_detect(cam, args, out, rep):
    """Is a camera there, and is it the Wide one the presets assume?"""
    info = Picamera2.global_camera_info()
    rep.data['global_camera_info'] = info
    for entry in info:
        rep.note('camera %s: %s' % (entry.get('Num'), entry.get('Model')))
    rep.check('camera present', len(info) > 0, '%d camera(s)' % len(info), '>=1',
              'rpicam-hello --list-cameras')

    props = dict(cam.camera_properties)
    rep.data['camera_properties'] = {k: str(v) for k, v in props.items()}
    model = str(props.get('Model', '?'))
    # A standard Module 3 reports imx708 and gives 66 deg where cameras.yaml claims 102.
    # Nothing downstream would notice: image_features is handed the hFOV, not asked for it.
    rep.check('sensor model', model == MODEL, model, MODEL, 'cameras.yaml:62')
    rep.note('sensor resolution %s, pixel array %s'
             % (cam.sensor_resolution, props.get('PixelArraySize')))


def t_modes(cam, args, out, rep):
    """Do the two presets correspond to real sensor modes at the rates claimed?"""
    modes = cam.sensor_modes
    rep.data['sensor_modes'] = [{k: str(v) for k, v in m.items()} for m in modes]
    print('    %-14s %-10s %-8s %-6s %s' % ('size', 'format', 'fps', 'bits', 'crop_limits'))
    for m in modes:
        print('    %-14s %-10s %-8.2f %-6s %s'
              % ('%dx%d' % tuple(m['size']), m['format'], m.get('fps', 0),
                 m.get('bit_depth', '?'), m.get('crop_limits', '?')))

    for spec in (PRIMARY, FALLBACK):
        label = '%dx%d' % tuple(spec['size'])
        mode = find_mode(modes, spec['size'])
        if mode is None:
            rep.check('mode %s exists' % label, False, 'absent', 'present',
                      'cameras.yaml %s' % spec['preset'])
            continue
        rep.check('mode %s exists' % label, True, 'present', 'present',
                  'cameras.yaml %s' % spec['preset'])
        got = float(mode.get('fps', 0.0))
        # A mismatch here means cameras.yaml was written against a different firmware or a
        # different bit depth, and every fps claim downstream inherits the error.
        rep.check('mode %s advertised rate' % label,
                  abs(got - spec['fps']) <= FPS_TOLERANCE * spec['fps'],
                  '%.2f fps' % got, '%.2f +/-%d%%' % (spec['fps'], FPS_TOLERANCE * 100),
                  'cameras.yaml %s fps' % spec['preset'])


def t_fov(cam, args, out, rep):
    """Is the full field of view actually being read out, or quietly cropped?"""
    modes = cam.sensor_modes
    mode = find_mode(modes, PRIMARY['size'])
    if mode is None:
        rep.check('fov: 2304x1296 mode', False, 'absent', 'present', 'cameras.yaml:65')
        return
    restart(cam, video_config(cam, PRIMARY, mode))
    time.sleep(1.0)
    md = cam.capture_metadata()
    props = cam.camera_properties

    areas = props.get('PixelArrayActiveAreas') or [(0, 0) + tuple(props['PixelArraySize'])]
    active = tuple(areas[0])
    crop = tuple(md.get('ScalerCrop', (0, 0, 0, 0)))
    rep.data['fov'] = {'active_area': active, 'scaler_crop': crop}
    fw = crop[2] / float(active[2]) if active[2] else 0.0
    fh = crop[3] / float(active[3]) if active[3] else 0.0
    rep.note('active area %s, ScalerCrop %s' % (active, crop))

    # crop: 1.0 in the preset IS the 102 degrees. A crop of 0.67 would silently be the
    # module3wide_1536 geometry (78.9 deg) while every consumer still assumed 102.
    rep.check('horizontal crop fraction', abs(fw - PRIMARY['crop']) <= CROP_TOLERANCE,
              '%.4f' % fw, '%.2f +/-%.2f' % (PRIMARY['crop'], CROP_TOLERANCE),
              'cameras.yaml:69 crop')
    rep.check('vertical crop fraction', abs(fh - PRIMARY['crop']) <= CROP_TOLERANCE,
              '%.4f' % fh, '%.2f +/-%.2f' % (PRIMARY['crop'], CROP_TOLERANCE),
              'cameras.yaml:69 crop')

    fx = fx_of(PRIMARY['size'][0], PRIMARY['hfov_deg'])
    rep.data['fov']['fx_px'] = fx
    rep.note('implied fx = %.1f px at %dx%d, %.1f deg hFOV (camera_presets.resolve)'
             % (fx, PRIMARY['size'][0], PRIMARY['size'][1], PRIMARY['hfov_deg']))
    rep.note('hardware.launch.py sizes fx from the width camera_node publishes (the render '
             'size by default) - this is the number the flown image_features gets')


def measure_rate(cam, spec, mode, frames, rep):
    """Two cadences for one mode: what the sensor did, and what a subscriber would see.

    capture_request/release is deliberately not capture_array: at 2304x1296 RGB a copy is
    ~9 MB, and timing the copy instead of the pipeline would measure the wrong thing. The
    copy cost is sampled separately so it is still visible.
    """
    restart(cam, video_config(cam, spec, mode))
    time.sleep(1.5)      # let AE/AWB settle; the first frames are not representative

    delivered = []
    sensor_ts = []
    exposures = []
    copy_ms = []
    actual_size = None
    t_prev = None
    for i in range(frames):
        req = cam.capture_request()
        now = time.monotonic()
        try:
            md = req.get_metadata()
            if t_prev is not None:
                delivered.append((now - t_prev) * 1000.0)
            t_prev = now
            if 'SensorTimestamp' in md:
                sensor_ts.append(md['SensorTimestamp'])
            if 'ExposureTime' in md:
                exposures.append(md['ExposureTime'])
            if i % 10 == 0:
                t0 = time.monotonic()
                arr = req.make_array('main')
                copy_ms.append((time.monotonic() - t0) * 1000.0)
                actual_size = (arr.shape[1], arr.shape[0])
        finally:
            req.release()

    sensor_iv = [(b - a) / 1e6 for a, b in zip(sensor_ts, sensor_ts[1:])]
    budget_ms = 1000.0 / spec['fps']
    stats = {
        'frames': frames,
        'delivered_hz': 1000.0 / mean(delivered) if delivered else 0.0,
        'sensor_hz': 1000.0 / mean(sensor_iv) if sensor_iv else 0.0,
        'p50_ms': pct(delivered, 0.50), 'p95_ms': pct(delivered, 0.95),
        'max_ms': max(delivered) if delivered else float('nan'),
        'frac_over_loop_budget': (len([d for d in delivered if d > 1000.0 / LOOP_HZ])
                                  / float(len(delivered))) if delivered else 1.0,
        'mean_exposure_us': mean(exposures) if exposures else float('nan'),
        'mean_copy_ms': mean(copy_ms) if copy_ms else float('nan'),
        'stream_size': actual_size,
        'frame_budget_ms': budget_ms,
    }
    rep.note('delivered %.2f Hz | sensor %.2f Hz | p50 %.2f ms  p95 %.2f ms  max %.2f ms'
             % (stats['delivered_hz'], stats['sensor_hz'], stats['p50_ms'], stats['p95_ms'],
                stats['max_ms']))
    rep.note('frames slower than the %.0f Hz loop budget: %.1f%% | mean exposure %.0f us '
             '| RGB copy %.1f ms'
             % (LOOP_HZ, 100.0 * stats['frac_over_loop_budget'], stats['mean_exposure_us'],
                stats['mean_copy_ms']))
    if actual_size and tuple(actual_size) != tuple(spec['size']):
        rep.note('WARNING: stream came back %s, not the requested %s - the mode was not '
                 'honoured' % (actual_size, tuple(spec['size'])))
    return stats


def t_rate(cam, args, out, rep):
    """The load-bearing test: does 2304x1296 clear the 50 Hz loop on this hardware?"""
    modes = cam.sensor_modes
    rep.data['rate'] = {}
    for spec in (PRIMARY, FALLBACK):
        label = '%dx%d' % tuple(spec['size'])
        mode = find_mode(modes, spec['size'])
        if mode is None:
            rep.check('rate %s' % label, False, 'mode absent', '>=%.1f Hz' % LOOP_HZ,
                      'camera_presets.py:35')
            continue
        print('    --- %s (%s) ---' % (label, spec['preset']))
        stats = measure_rate(cam, spec, mode, args.frames, rep)
        rep.data['rate'][spec['preset']] = stats

        ok = stats['delivered_hz'] >= LOOP_HZ
        rep.check('rate %s sustained' % label, ok, '%.2f Hz' % stats['delivered_hz'],
                  '>=%.1f Hz' % LOOP_HZ, 'camera_presets.py:35 LOOP_HZ')
        # Advertised-vs-achieved is a separate question from clearing the loop: a mode that
        # runs at 52 Hz passes the loop but means cameras.yaml's 56.03 is optimistic.
        rep.check('rate %s vs preset' % label,
                  stats['delivered_hz'] >= (1 - 0.05) * spec['fps'],
                  '%.2f Hz' % stats['delivered_hz'], '>=%.2f Hz' % (0.95 * spec['fps']),
                  'cameras.yaml %s fps' % spec['preset'])

        if not ok:
            rep.note('diagnosis, in the order worth checking:')
            if stats['mean_exposure_us'] > 1000.0 * stats['frame_budget_ms']:
                rep.note('  * exposure %.0f us exceeds the %.2f ms frame budget - too dark. '
                         'Re-run in bright light before believing this failure.'
                         % (stats['mean_exposure_us'], stats['frame_budget_ms']))
            if stats['stream_size'] and tuple(stats['stream_size']) != tuple(spec['size']):
                rep.note('  * the sensor mode was not honoured (see the size warning above)')
            rep.note('  * CPU contention: nothing else should be running; check the '
                     'metadata subtest for thermal throttling')
            rep.note('  * a 50 Hz loop fed slower than 50 Hz repeats frames silently - this '
                     'is exactly what cameras.yaml:11 documents for Module 2')


def t_still(cam, args, out, rep):
    """Full-resolution capture - proves the stills path and the whole 4608x2592 readout."""
    try:
        if cam.started:
            cam.stop()
    except Exception:
        pass
    cam.configure(cam.create_still_configuration())
    cam.start()
    time.sleep(1.5)
    path = os.path.join(out, 'still_full_res.jpg')
    md = cam.capture_file(path)
    size = os.path.getsize(path)
    rep.data['still'] = {'path': path, 'bytes': size,
                         'metadata': {k: str(v) for k, v in md.items()}}
    rep.note('wrote %s (%.1f MB), exposure %s us, gain %s'
             % (path, size / 1e6, md.get('ExposureTime'), md.get('AnalogueGain')))
    rep.check('full-res still written', size > 100000, '%d bytes' % size, '>100 kB',
              'sensor_resolution %s' % (cam.sensor_resolution,))


def t_focus(cam, args, out, rep):
    """Autofocus, then a manual sweep across the lens' real range.

    The Wide module's near limit (~5 cm) is closer than the standard Module 3's (~10 cm),
    and it shows up as a larger maximum LensPosition in dioptres. Read the range rather
    than hardcoding it - it is the honest way to ask the lens what it can do.
    """
    modes = cam.sensor_modes
    mode = find_mode(modes, PRIMARY['size'])
    spec = PRIMARY if mode else FALLBACK
    mode = mode or find_mode(modes, FALLBACK['size'])
    restart(cam, video_config(cam, spec, mode))
    time.sleep(1.0)

    limits = cam.camera_controls.get('LensPosition')
    if limits is None:
        rep.check('lens is motorised', False, 'no LensPosition control', 'present',
                  'Camera Module 3 has autofocus')
        return
    lo, hi, default = limits[0], limits[1], limits[2]
    near_cm = 100.0 / hi if hi else float('inf')
    rep.data['focus'] = {'lens_position_limits': [lo, hi, default], 'near_limit_cm': near_cm}
    rep.note('LensPosition %.2f..%.2f dioptres (default %s) -> near limit ~%.1f cm'
             % (lo, hi, default, near_cm))
    rep.check('lens is motorised', True, '%.1f..%.1f dioptres' % (lo, hi), 'present',
              'IMX708 autofocus')

    t0 = time.monotonic()
    try:
        cam.set_controls({'AfMode': libcontrols.AfModeEnum.Auto})
        ok = cam.autofocus_cycle()
    except Exception as exc:
        ok = False
        rep.note('autofocus_cycle raised: %s' % exc)
    elapsed = time.monotonic() - t0
    md = cam.capture_metadata()
    state = md.get('AfState')
    focused = ok or state == libcontrols.AfStateEnum.Focused
    rep.data['focus']['af_cycle_s'] = elapsed
    rep.data['focus']['af_state'] = str(state)
    rep.data['focus']['settled_lens_position'] = md.get('LensPosition')
    rep.check('autofocus converges', focused, 'AfState=%s in %.2f s' % (state, elapsed),
              'Focused', 'libcamera AfStateEnum')
    rep.note('settled at LensPosition %s (~%.1f cm)'
             % (md.get('LensPosition'),
                100.0 / md['LensPosition'] if md.get('LensPosition') else float('inf')))

    # The sweep. Manual mode, one JPEG per step, named by the distance it focuses at, so
    # the near end can be checked against a ruler rather than trusted.
    cam.set_controls({'AfMode': libcontrols.AfModeEnum.Manual})
    steps = args.focus_steps
    saved = []
    for i in range(steps):
        d = lo + (hi - lo) * i / float(steps - 1) if steps > 1 else default
        cam.set_controls({'LensPosition': d})
        time.sleep(0.6)                      # the VCM needs time to settle before the shot
        cm = 100.0 / d if d > 0 else float('inf')
        name = 'focus_%05.2fdpt_%s.jpg' % (d, 'inf' if d <= 0 else '%03.0fcm' % cm)
        path = os.path.join(out, name)
        cam.capture_file(path)
        saved.append({'dioptres': d, 'distance_cm': cm, 'path': path})
        rep.note('%.2f dioptres -> %s -> %s'
                 % (d, 'infinity' if d <= 0 else '%.1f cm' % cm, name))
    rep.data['focus']['sweep'] = saved
    rep.check('focus sweep captured', len(saved) == steps, '%d images' % len(saved),
              '%d images' % steps, 'manual LensPosition sweep')
    cam.set_controls({'AfMode': libcontrols.AfModeEnum.Continuous})


def t_video(cam, args, out, rep):
    """A short clip. Sanity only - deliberately NOT a performance gate.

    The Pi 5 has no hardware H.264 encoder; picamera2 encodes in software here. The IBVS
    path never encodes anything - image_features consumes raw frames - so a slow encode
    says nothing about whether the aircraft can fly. The rate subtest is the one that does.
    """
    try:
        cfg = cam.create_video_configuration(main={'size': (1920, 1080), 'format': 'YUV420'},
                                             controls={'FrameRate': 30.0}, buffer_count=6)
        restart(cam, cfg)
        time.sleep(1.0)
        path = os.path.join(out, 'clip.mp4')
        encoder = H264Encoder(bitrate=10000000)
        try:
            output = FfmpegOutput(path)
        except Exception:
            path = os.path.join(out, 'clip.h264')
            output = FileOutput(path)
        rep.note('recording %.1f s at 1920x1080 (software encode on the Pi 5)'
                 % args.video_seconds)
        cam.start_recording(encoder, output)
        time.sleep(args.video_seconds)
        cam.stop_recording()
    except Exception as exc:
        rep.check('video clip written', False, 'raised %s' % exc, 'a playable file',
                  'picamera2 H264Encoder')
        return
    size = os.path.getsize(path) if os.path.exists(path) else 0
    rep.data['video'] = {'path': path, 'bytes': size, 'seconds': args.video_seconds}
    rep.note('wrote %s (%.1f MB)' % (path, size / 1e6))
    rep.check('video clip written', size > 100000, '%d bytes' % size, '>100 kB',
              'sanity only, not a gate')
    # start_recording leaves the camera running; hand the next subtest a clean state.
    try:
        if cam.started:
            cam.stop()
    except Exception:
        pass


def t_metadata(cam, args, out, rep):
    """The numbers that explain a failure elsewhere, especially a rate test that drifts."""
    modes = cam.sensor_modes
    mode = find_mode(modes, PRIMARY['size']) or modes[-1]
    restart(cam, video_config(cam, PRIMARY, mode))
    time.sleep(1.5)
    md = cam.capture_metadata()
    keys = ('ExposureTime', 'AnalogueGain', 'DigitalGain', 'LensPosition', 'AfState',
            'ColourTemperature', 'SensorTemperature', 'FrameDuration', 'ScalerCrop',
            'SensorTimestamp', 'Lux')
    dump = {}
    for k in keys:
        if k in md:
            dump[k] = str(md[k])
            rep.note('%-18s %s' % (k, md[k]))
    rep.data['metadata'] = dump

    temp = vcgencmd('measure_temp')
    throttled = vcgencmd('get_throttled')
    rep.data['soc'] = {'measure_temp': temp, 'get_throttled': throttled}
    rep.note('SoC %s, %s' % (temp, throttled))
    # 0x0 means never throttled since boot. Anything else and a rate test that passed cold
    # will not still pass after ten minutes of flight prep.
    rep.check('SoC not throttled', throttled.endswith('=0x0') or 'unavailable' in throttled,
              throttled, 'throttled=0x0', 'vcgencmd get_throttled')
    rep.check('metadata readable', len(dump) >= 5, '%d fields' % len(dump), '>=5 fields',
              'libcamera request metadata')


SUBTESTS = {'detect': t_detect, 'modes': t_modes, 'fov': t_fov, 'rate': t_rate,
            'still': t_still, 'focus': t_focus, 'video': t_video, 'metadata': t_metadata}


# --- calibration capture ----------------------------------------------------------------

def run_calib(cam, args, out):
    """Collect checkerboard stills at the flown mode. Capture only - no solving here.

    They must come from 2304x1296 with the full crop, because that is the geometry
    cameras.yaml:73 describes; coefficients fitted to a different mode do not transfer.
    """
    modes = cam.sensor_modes
    mode = find_mode(modes, PRIMARY['size'])
    if mode is None:
        raise SystemExit('the 2304x1296 sensor mode is missing - calibrate nothing until '
                         'the modes subtest passes')
    cfg = cam.create_still_configuration(main={'size': tuple(PRIMARY['size'])})
    restart(cam, cfg)
    time.sleep(1.5)
    cam.set_controls({'AfMode': libcontrols.AfModeEnum.Continuous})

    folder = os.path.join(out, 'calib')
    os.makedirs(folder, exist_ok=True)
    print('capturing %d frames at %dx%d, %.1f s apart.'
          % (args.calib, PRIMARY['size'][0], PRIMARY['size'][1], args.interval))
    print('Re-pose the checkerboard between shots: fill the frame, tilt it, and get it into '
          'the corners - the barrel term lives at the edges.')
    for i in range(args.calib):
        for remaining in range(int(args.interval), 0, -1):
            print('  %2d/%d in %d...' % (i + 1, args.calib, remaining), end='\r')
            time.sleep(1.0)
        path = os.path.join(folder, 'calib_%03d.jpg' % i)
        cam.capture_file(path)
        print('  %2d/%d -> %s        ' % (i + 1, args.calib, path))
    print('')
    print('Next, off the Pi:')
    print('  run cv2.findChessboardCorners + cv2.calibrateCamera over %s' % folder)
    print('  then replace the ASSUMED distortion at cameras.yaml:73')
    print('      distortion: %s' % ASSUMED_DISTORTION)
    print('  with the fitted [k1, k2, p1, p2, k3], and drop the "ASSUMPTION" comment above '
          'it.')
    print('  module3wide_2304_ideal (cameras.yaml:77) is the zero-distortion control arm '
          'and stays as it is.')


# --- main -------------------------------------------------------------------------------

def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--only', default='', help='comma-separated subtests: %s'
                   % ', '.join(TESTS))
    p.add_argument('--skip', default='', help='comma-separated subtests to skip')
    p.add_argument('--frames', type=int, default=300,
                   help='frames per mode in the rate test (default 300)')
    p.add_argument('--focus-steps', type=int, default=8,
                   help='LensPosition steps in the focus sweep (default 8)')
    p.add_argument('--video-seconds', type=float, default=5.0)
    p.add_argument('--out', default='',
                   help='results directory (default results_<timestamp>)')
    p.add_argument('--preview', action='store_true', help='show a live window while testing')
    p.add_argument('--calib', type=int, default=0,
                   help='capture N checkerboard stills instead of running the tests')
    p.add_argument('--interval', type=float, default=3.0,
                   help='seconds between --calib captures (default 3)')
    p.add_argument('--doctor', action='store_true',
                   help='run the environment preflight and stop (needs no camera)')
    p.add_argument('--skip-doctor', action='store_true',
                   help='go straight to the tests without the preflight')
    args = p.parse_args(argv)

    for name in [n for n in (args.only + ',' + args.skip).split(',') if n]:
        if name not in TESTS:
            p.error('%r is not a subtest; choose from %s' % (name, ', '.join(TESTS)))
    return args


def main(argv):
    args = parse_args(argv)
    out = args.out or ('results_%s' % datetime.now().strftime('%Y%m%d-%H%M%S'))
    if not args.doctor:          # --doctor writes nothing; it has to be safe to just run
        os.makedirs(out, exist_ok=True)

    selected = [t for t in TESTS
                if (not args.only or t in args.only.split(','))
                and t not in args.skip.split(',')]

    print('Camera Module 3 Wide bench test')
    print('checked against cameras.yaml: %s = %dx%d @ %.2f fps, %.1f deg, crop %.1f'
          % (PRIMARY['preset'], PRIMARY['size'][0], PRIMARY['size'][1], PRIMARY['fps'],
             PRIMARY['hfov_deg'], PRIMARY['crop']))
    print('the bar it has to clear: %.0f Hz (image_features / fixed_eso)' % LOOP_HZ)
    if not args.doctor:
        print('results -> %s' % os.path.abspath(out))
    print('')

    # The preflight runs first because on Ubuntu every subtest below fails for one reason -
    # a libcamera without the PiSP pipeline handler - and a wall of camera errors hides it.
    blocking = 0
    if args.doctor or not args.skip_doctor:
        blocking = doctor()
        print('')
    if args.doctor:
        return 1 if blocking else 0

    if PICAMERA2_ERROR is not None:
        print('cannot run the tests: picamera2 is not importable (%s).' % PICAMERA2_ERROR)
        print('The preflight above says which step is missing. On Ubuntu 24.04 start with')
        print('    bash tools/setup_pi5_ubuntu_camera.sh')
        return 2

    cam = Picamera2()
    if args.preview:
        start_preview(cam)

    rep = Report()
    try:
        if args.calib:
            run_calib(cam, args, out)
            return 0

        for name in selected:
            print('')
            print('--- %s %s' % (name, '-' * (70 - len(name))))
            try:
                SUBTESTS[name](cam, args, out, rep)
            except Exception as exc:
                rep.check('%s subtest' % name, False, 'raised %r' % exc, 'completes',
                          'unexpected error')
        rep.skipped = [t for t in TESTS if t not in selected]
        rep.table()
    finally:
        try:
            if cam.started:
                cam.stop()
        except Exception:
            pass
        cam.close()
        if rep.checks:
            path = os.path.join(out, 'results.json')
            with open(path, 'w') as fh:
                json.dump({'checks': rep.checks, 'data': rep.data,
                           'spec': {'LOOP_HZ': LOOP_HZ, 'primary': PRIMARY,
                                    'fallback': FALLBACK, 'model': MODEL}},
                          fh, indent=2, default=str)
            print('wrote %s' % path)

    return 1 if rep.failed() else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
