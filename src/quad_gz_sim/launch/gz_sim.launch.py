"""Gazebo itself: the world, the ros_gz bridge and the pose broadcaster.

  ros2 launch quad_gz_sim gz_sim.launch.py [headless:=true] [plant:=analytic|gazebo]
                                           [camera:=module2_1640] [target_scale:=1.0]
                                           [marker_dict:=7x7] [camera_rate:=0.0]

plant starts no node here; it selects how the world is derived. See _world_for().

camera, target_scale and marker_dict do the same for the models: _models_for() writes derived
copies of F450_base and aruco_target into a scratch tree that is prepended to
GZ_SIM_RESOURCE_PATH, so model:// still resolves and the originals are never touched. The
camera presets live in config/cameras.yaml, which the offline footprint calculator reads too -
one table, so the rendered geometry and the calculator cannot disagree.

The matching intrinsics have to reach image_features as well, or the feature model is computed
against a camera that was not rendered. That is the caller's job: quad_px4/sitl.launch.py and
quad_utils/sim.launch.py resolve the preset once and pass the numbers to both.
"""
import importlib.util
import math
import os
import re
import shutil
import signal

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'quad_gz_sim'

PLANTS = ('analytic', 'gazebo', 'px4')

# Marker dictionaries image_features can be pointed at, and the meshes directory that carries
# the matching textures. Same IDs either way, so the corner-ordering branches never change.
MARKER_DICTS = {'7x7': 'meshes', '4x4': 'meshes_4x4'}


def _presets():
    """camera_presets.py, which lives beside this file. share/<pkg>/launch is not on
    sys.path, so it is loaded by path rather than imported."""
    path = os.path.join(get_package_share_directory(PKG), 'launch', 'camera_presets.py')
    spec = importlib.util.spec_from_file_location('camera_presets', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def generate_launch_description():
    share = get_package_share_directory(PKG)
    presets = _presets()
    world_file = os.path.join(share, 'worlds', 'ibvs.sdf')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    args = [
        DeclareLaunchArgument('headless', default_value='false'),
        # Must match <world name=...> in worlds/ibvs.sdf: selects the gz topic
        # /world/<world>/set_pose_vector the broadcaster publishes on.
        DeclareLaunchArgument('world', default_value='ibvs'),
        # analytic = the paper's ROS-side integrator; gazebo = DART; px4 = DART with PX4
        # SITL owning allocation and the inner loop.
        DeclareLaunchArgument('plant', default_value='analytic'),
        # Camera module AND sensor mode; see config/cameras.yaml. Not every module reaches the
        # 50 Hz image_features loop at full field of view, which is why modes are separate
        # presets rather than a resolution argument.
        DeclareLaunchArgument('camera', default_value=presets.DEFAULT_CAMERA),
        # Scales the ArUco target. Smaller markers buy field-of-view slack and cost decode
        # pixels; footprint.py prints both sides of that trade.
        DeclareLaunchArgument('target_scale', default_value='1.0'),
        DeclareLaunchArgument('marker_dict', default_value='7x7',
                              description='7x7 (the thesis, and every recorded bag) or 4x4, '
                                          'which decodes at about two thirds the pixel size'),
        # Above zero, overrides the camera sensor's update_rate. Use it to fly the real sensor
        # mode's frame rate against the 50 Hz loop instead of the sim's free 50 fps.
        DeclareLaunchArgument('camera_rate', default_value='0.0'),
    ]

    models = os.path.join(get_package_share_directory('quad_description'), 'models')
    cameras = presets.table()

    # Resolves model://F450 and model://aruco_target. The derived tree comes first so the
    # rewritten F450_base and aruco_target win; everything else falls through to the originals.
    derived_models = os.path.join(share, 'worlds', '.models')
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', derived_models + os.pathsep + models)

    # Where gz-sim looks for libBodyWrenchSystem.so (CMakeLists installs it to lib/).
    plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(get_package_prefix(PKG), 'lib'))

    # Without this, gz-sim's sensor rendering picks the Mesa ICD and falls back to software
    # (llvmpipe) even though the GPU is present.
    egl_vendor = [
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
                               '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
    ]

    def _reap_orphans(context, *a, **k):
        """Kill gz sim processes orphaned by a previous run (they hold a GPU context).

        Requires all three of: 'gz sim' in the cmdline, this package's name in it, and
        parent pid 1 - so a live simulation, or a Gazebo running for anything else, is
        never touched. PKG reaches the cmdline via the derived world path.
        """
        killed = []
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            try:
                with open('/proc/%s/cmdline' % entry, 'rb') as fh:
                    cmd = fh.read().decode('utf-8', 'replace').replace('\0', ' ')
                if 'gz sim' not in cmd or PKG not in cmd:
                    continue
                # "pid (comm) state ppid ..."; comm may contain spaces, so split last ')'.
                with open('/proc/%s/stat' % entry) as fh:
                    ppid = int(fh.read().rsplit(')', 1)[1].split()[1])
                if ppid != 1:
                    continue
                os.kill(int(entry), signal.SIGKILL)
                killed.append(entry)
            except (OSError, ValueError, IndexError):
                continue

        if not killed:
            return []
        return [LogInfo(msg='Reaped orphaned gz sim from a previous run: pid ' +
                            ', '.join(killed))]

    def _select(text, plant, what):
        """Keep each ONLY:<plants> block only if `plant` is in its list. Raises if no marker
        is found, so an unfiltered file cannot silently run the wrong plant."""
        seen = [False]

        def keep(m):
            seen[0] = True
            return m.group(0) if plant in m.group(1).split(',') else ''

        # [a-z0-9,] rather than [a-z,]: 'px4' has a digit in it.
        out = re.sub(r'[ \t]*<!-- ONLY:([a-z0-9,]+) BEGIN -->.*?<!-- ONLY END -->\n',
                     keep, text, flags=re.DOTALL)
        if not seen[0]:
            raise RuntimeError(
                '%s has no ONLY: markers, so gz_sim.launch.py cannot select the plugins '
                'for plant:=%s. Fix the file or this launch file before running.'
                % (what, plant))
        return out

    def _sub_once(text, pattern, repl, what):
        """re.sub that insists on exactly one match, so a renamed tag fails loudly instead of
        silently rendering the wrong camera."""
        out, n = re.subn(pattern, repl, text, count=1)
        if n != 1:
            raise RuntimeError(
                'gz_sim.launch.py could not find %s in the model SDF. The tag was renamed or '
                'removed; fix the SDF or this launch file before running.' % what)
        return out

    def _derive_model(name, rewrite, meshes_from=None):
        """Write a rewritten copy of models/<name>/ into the derived tree.

        model.sdf is rewritten, model.config copied, and meshes/ symlinked rather than copied -
        F450_base's are 4 MB and none of them change. The mesh URIs are model://<name>/meshes/...
        so they resolve through the derived directory and land on the symlink.
        """
        src = os.path.join(models, name)
        dst = os.path.join(derived_models, name)
        shutil.rmtree(dst, ignore_errors=True)
        os.makedirs(dst)
        with open(os.path.join(src, 'model.sdf')) as fh:
            sdf = fh.read()
        with open(os.path.join(dst, 'model.sdf'), 'w') as fh:
            fh.write(rewrite(sdf))
        shutil.copy(os.path.join(src, 'model.config'), dst)
        meshes = os.path.join(src, meshes_from or 'meshes')
        if not os.path.isdir(meshes):
            raise RuntimeError('%s does not exist. Generate it with '
                               'quad_description/scripts/make_markers.py.' % meshes)
        os.symlink(meshes, os.path.join(dst, 'meshes'))

    def _models_for(camera, target_scale, marker_dict, camera_rate):
        """Derive F450_base (camera preset) and aruco_target (scale, marker dictionary)."""
        if camera not in cameras:
            raise RuntimeError('camera:=%s is not one of %s.'
                               % (camera, ', '.join(cameras)))
        if marker_dict not in MARKER_DICTS:
            raise RuntimeError('marker_dict:=%s is not one of %s.'
                               % (marker_dict, ', '.join(MARKER_DICTS)))
        if target_scale <= 0:
            raise RuntimeError('target_scale:=%g must be positive.' % target_scale)
        cam = cameras[camera]
        width, height = cam['render']
        k1, k2, p1, p2, k3 = cam['distortion']

        def camera_block(sdf):
            # Scoped to the camera1 sensor: update_rate appears on the IMU, magnetometer,
            # barometer and NavSat too, and none of those may move with the camera.
            m = re.search(r"<sensor name='camera1'.*?</sensor>", sdf, re.DOTALL)
            if m is None:
                raise RuntimeError("F450_base/model.sdf has no <sensor name='camera1'> block; "
                                   'gz_sim.launch.py cannot apply camera:=%s.' % camera)
            blk = m.group(0)
            blk = _sub_once(blk, r'<horizontal_fov>[^<]*</horizontal_fov>',
                            '<horizontal_fov>%.9f</horizontal_fov>'
                            % math.radians(cam['hfov_deg']), 'horizontal_fov')
            blk = _sub_once(blk, r'<width>\d+</width>', '<width>%d</width>' % width, 'width')
            blk = _sub_once(blk, r'<height>\d+</height>', '<height>%d</height>' % height,
                            'height')
            if any(c != 0.0 for c in cam['distortion']):
                # gz-rendering applies this as a post-render pass, so detectMarkers sees the
                # bowed image - which is the point: point undistortion downstream cannot
                # rescue a marker the detector never found.
                blk = _sub_once(
                    blk, r'(\s*)</camera>',
                    r'\1  <distortion>\1    <k1>%g</k1><k2>%g</k2><k3>%g</k3>'
                    r'\1    <p1>%g</p1><p2>%g</p2>\1    <center>0.5 0.5</center>'
                    r'\1  </distortion>\1</camera>' % (k1, k2, k3, p1, p2), '</camera>')
            if camera_rate > 0:
                blk = _sub_once(blk, r'<update_rate>\d+(\.\d+)?</update_rate>',
                                '<update_rate>%g</update_rate>' % camera_rate, 'update_rate')
            return sdf[:m.start()] + blk + sdf[m.end():]

        def target_block(sdf):
            if target_scale == 1.0:
                return sdf
            sdf = _sub_once(sdf, r'<scale>[^<]*</scale>',
                            '<scale>%g %g %g</scale>'
                            % (target_scale, target_scale, target_scale), 'mesh <scale>')
            return _sub_once(sdf, r'<size>[^<]*</size>',
                             '<size>%g %g 0.001</size>'
                             % (0.90 * target_scale, 0.75 * target_scale), 'collision <size>')

        _derive_model('F450_base', camera_block)
        _derive_model('aruco_target', target_block, MARKER_DICTS[marker_dict])

    def _world_for(plant):
        """Derive the world for `plant`: filter ONLY blocks, and zero gravity for analytic."""
        with open(world_file) as fh:
            sdf = fh.read()

        out = _select(sdf, plant, 'worlds/ibvs.sdf')
        if plant == 'analytic':
            gravity_off = out.replace('<gravity>0 0 -9.81</gravity>',
                                      '<gravity>0 0 0</gravity>')
            if gravity_off == out:
                raise RuntimeError(
                    'worlds/ibvs.sdf no longer has the gravity tag gz_sim.launch.py '
                    'rewrites for plant:=analytic. Fix one or the other before running.')
            out = gravity_off

        # Alongside the original so model:// still resolves.
        derived = os.path.join(os.path.dirname(world_file), '.ibvs_%s.sdf' % plant)
        with open(derived, 'w') as fh:
            fh.write(out)
        return derived

    def _gz(context, *a, **k):
        headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
        plant = LaunchConfiguration('plant').perform(context).lower()
        if plant not in PLANTS:
            raise RuntimeError('plant:=%s is not one of %s.' % (plant, ', '.join(PLANTS)))
        camera = LaunchConfiguration('camera').perform(context)
        _models_for(camera,
                    float(LaunchConfiguration('target_scale').perform(context)),
                    LaunchConfiguration('marker_dict').perform(context),
                    float(LaunchConfiguration('camera_rate').perform(context)))
        cam = cameras[camera]
        # Logged because the sensor mode is a hardware constraint the sim otherwise hides:
        # Module 2's full-FOV mode runs at 41.85 fps against a 50 Hz loop.
        rate = float(LaunchConfiguration('camera_rate').perform(context))
        # -r starts the world running, so nothing has to unpause physics.
        flags = '-r -v3 -s --headless-rendering ' if headless else '-r -v3 '
        return [LogInfo(
            msg='camera %s: %.1f deg hFOV, render %dx%d, hardware mode %dx%d @ %.1f fps%s%s'
                % (camera, cam['hfov_deg'], cam['render'][0], cam['render'][1],
                   cam['native'][0], cam['native'][1], cam['fps'],
                   '' if cam['fps'] >= 50.0 else '  <- BELOW the 50 Hz image_features loop',
                   '' if rate <= 0 else '; rendering at %g fps' % rate)),
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': flags + _world_for(plant),
                              'gz_version': '8'}.items(),
            )]

    def _bridge(context, *a, **k):
        # plant:=px4 uses a config without the /quad_thrust and /quad_torques ROS_TO_GZ
        # entries: PX4 drives the rotors, and anything BodyWrench applied to base_link would
        # be added on top of that rather than instead of it.
        plant = LaunchConfiguration('plant').perform(context).lower()
        cfg = 'bridge_px4.yaml' if plant == 'px4' else 'bridge.yaml'
        return [Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='ibvs_bridge',
            output='screen',
            parameters=[{'config_file': os.path.join(share, 'config', cfg)}],
        )]

    def _distort(context, *a, **k):
        """Apply the preset's lens distortion, which gz-sim's ogre2 renderer will not.

        The SDF <distortion> element parses and does nothing: DistortionPass exists only in the
        ogre (v1) engine. Verified rather than assumed - k1 = -0.30 on the F450 camera gives a
        byte-identical frame. So the warp happens here, and image_features is remapped onto the
        distorted topic by the caller. A zero coefficient vector makes the node a pass-through,
        which keeps the topic plumbing identical whether distortion is on or off.
        """
        camera = LaunchConfiguration('camera').perform(context)
        cam = presets.resolve(camera)
        if not any(cam['distortion']):
            return []
        return [Node(
            package=PKG, executable='camera_distort', name='camera_distort', output='screen',
            parameters=[{'use_sim_time': True,
                         'camera_hfov': cam['hfov'],
                         'camera_width': cam['width'],
                         'camera_height': cam['height'],
                         'camera_distortion': cam['distortion']}])]

    def _broadcaster(context, *a, **k):
        # Under plant:=gazebo physics owns the quad's pose; the broadcaster keeps the target
        # and the cosmetic rotor spin.
        gazebo = LaunchConfiguration('plant').perform(context).lower() != 'analytic'
        return [Node(
            package=PKG, executable='gz_pose_broadcaster', name='gz_pose_broadcaster',
            output='screen', parameters=[{'world': LaunchConfiguration('world'),
                                          'teleport_quad': not gazebo,
                                          'use_sim_time': True}],
        )]

    return LaunchDescription(args + egl_vendor + [
        resource_path,
        plugin_path,
        OpaqueFunction(function=_reap_orphans),   # before Gazebo starts
        OpaqueFunction(function=_gz),
        OpaqueFunction(function=_bridge),
        OpaqueFunction(function=_distort),
        OpaqueFunction(function=_broadcaster),
    ])
