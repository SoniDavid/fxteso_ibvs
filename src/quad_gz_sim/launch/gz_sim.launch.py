"""Gazebo itself: the world, the ros_gz bridge and the pose broadcaster.

  ros2 launch quad_gz_sim gz_sim.launch.py [headless:=true] [plant:=analytic|gazebo]

plant starts no node here; it selects how the world is derived. See _world_for().
"""
import os
import re
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


def generate_launch_description():
    share = get_package_share_directory(PKG)
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
    ]

    # Resolves model://F450 and model://aruco_target, which live in quad_description.
    resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('quad_description'), 'models'))

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
        # -r starts the world running, so nothing has to unpause physics.
        flags = '-r -v3 -s --headless-rendering ' if headless else '-r -v3 '
        return [IncludeLaunchDescription(
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
        OpaqueFunction(function=_broadcaster),
    ])
