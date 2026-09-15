"""The Raspberry Pi Camera Module 3 Wide driver on /quad/camera/image_raw.

Included by quad_px4's hardware.launch.py and quad_cam's bench.launch.py, which resolve the camera
preset once and give this driver and image_features the same size and QoS. Run it alone only to
check the camera (probe_latency, probe_detect).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# name -> (default, type)
_ARGS = {
    'width': ('1152', int),
    'height': ('648', int),
    'sensor_width': ('2304', int),
    'sensor_height': ('1296', int),
    # 0.0 = the sensor mode's max rate (~56 Hz), above the 50 Hz loop so frames stay fresh.
    'framerate': ('0.0', float),
    'image_topic': ('/quad/camera/image_raw', str),
    # 'sensor_data' (BEST_EFFORT) pairs with image_features' sensor_qos:=true; the launchers pass it.
    'image_qos': ('reliable', str),
    'frame_id': ('camera_optical_frame', str),
    # The imx708_wide reports Rotation: 180 and picamera2 does not apply it. Verify on a known scene.
    'hflip': ('true', bool),
    'vflip': ('true', bool),
    'af_mode': ('continuous', str),
    'lens_position': ('0.83', float),
    'publish_camera_info': ('false', bool),
    'camera_info_file': ('', str),
    'camera_info_topic': ('/quad/camera/camera_info', str),
    'watchdog_timeout_s': ('0.5', float),
    'restart_cooldown_s': ('10.0', float),
    'failures_before_restart': ('5', int),
}


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v[0]) for k, v in _ARGS.items()]
    # Pin camera_node to a core, e.g. '2'; contention otherwise reads as capture timeouts.
    args.append(DeclareLaunchArgument('cpu_affinity', default_value=''))
    params = {
        k: ParameterValue(LaunchConfiguration(k), value_type=v[1])
        for k, v in _ARGS.items()
    }

    def _node(context, *a, **k):
        cpu = LaunchConfiguration('cpu_affinity').perform(context)
        kwargs = {'prefix': 'taskset -c ' + cpu} if cpu else {}
        # FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA comes from the including launch, or camera_node itself.
        return [Node(
            package='quad_cam',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[params],
            **kwargs,
        )]

    return LaunchDescription(args + [OpaqueFunction(function=_node)])
