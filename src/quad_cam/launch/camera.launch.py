"""Bring up the real Raspberry Pi camera on /quad/camera/image_raw.

Runs alongside the estimation chain on the aircraft. The camera geometry here is the
`module3wide_2304` preset from quad_gz_sim/config/cameras.yaml rendered at 1152x648;
pair it with:

    ros2 launch quad_control estimation.launch.py use_sim_time:=false \\
        camera_hfov:=1.780236 camera_width:=1152 camera_height:=648 \\
        camera_distortion:="[-0.30, 0.10, 0.0, 0.0, -0.02]" aD:=<measured from /a_value>

Unifying the two into one hardware.launch.py is a separate task (see
fxteso_ibvs_experiments/reference/hardware-bringup.md).
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
    # 0.0 = the sensor mode's max rate (~56 Hz). Kept above the 50 Hz image_features
    # loop on purpose so the depth-1 consumer always has a fresh frame.
    'framerate': ('0.0', float),
    'image_topic': ('/quad/camera/image_raw', str),
    # 'reliable' is the drop-in match for image_features today; 'sensor_data' is correct
    # once image_features gains an image_qos parameter.
    'image_qos': ('reliable', str),
    'frame_id': ('camera_optical_frame', str),
    # The imx708_wide reports Rotation: 180; picamera2 does not auto-apply it. Verify
    # against a known scene and flip if the image is upside-down.
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
    # Pin camera_node to a CPU core, e.g. '2' - empty (default) leaves it unpinned. Under
    # a busy full graph this stops the OS scheduler from delaying picamera2's own
    # completion thread, which otherwise reads as capture timeouts / restart-loop churn.
    args.append(DeclareLaunchArgument('cpu_affinity', default_value=''))
    params = {
        k: ParameterValue(LaunchConfiguration(k), value_type=v[1])
        for k, v in _ARGS.items()
    }

    def _node(context, *a, **k):
        cpu = LaunchConfiguration('cpu_affinity').perform(context)
        kwargs = {'prefix': 'taskset -c ' + cpu} if cpu else {}
        return [Node(
            package='quad_cam',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[params],
            # Fast DDS's default shared-memory segment is 512 KiB; a 1152x648 bgr8 frame
            # is 2.24 MB. Under CPU contention (a busy subscriber like image_features)
            # the oversized message can't keep up with the small segment and frames are
            # silently lost well before they'd ever hit the OS UDP layer - observed as
            # the publisher reporting a healthy fps while every subscriber sees ~1-2 Hz.
            # eProsima's built-in LARGE_DATA transport profile raises the segment (and
            # the UDP fallback's max message size) to fit. Every participant on this
            # graph needs the same setting - see estimation.launch.py / control.launch.py.
            additional_env={'FASTDDS_BUILTIN_TRANSPORTS': 'LARGE_DATA'},
            **kwargs,
        )]

    return LaunchDescription(args + [OpaqueFunction(function=_node)])
