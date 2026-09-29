"""The plant. Both backends publish the same five state topics, so nothing downstream
knows which one ran.

  plant:=analytic  uav_dynamics integrates the aircraft; Gazebo is a camera only.
  plant:=gazebo    DART integrates it; gz_state_adapter converts the odometry to NED.

plant:=px4 is not served from here: PX4 SITL needs its own process and agent alongside the
state adapter, so quad_px4/launch/sitl.launch.py replaces this file wholesale.
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'quad_gz_sim'


def _airframe():
    """quad_description's airframe.py, loaded by path: share/<pkg>/launch is not on sys.path."""
    path = os.path.join(get_package_share_directory('quad_description'), 'launch',
                        'airframe.py')
    spec = importlib.util.spec_from_file_location('airframe', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def generate_launch_description():
    args = [
        DeclareLaunchArgument('plant', default_value='analytic'),
        # Debug only: false feeds a wrapped attitude, as a quaternion source would.
        DeclareLaunchArgument('unwrap_attitude', default_value='true'),
        # analytic only. 4.0 is the thesis' start; 2.5 matches zD, and so matches the small
        # initial estimation error that px4's takeoff gate produces.
        DeclareLaunchArgument('start_altitude', default_value='4.0'),
        # analytic only; gazebo's is in the derived SDF. Both come from quad_description's
        # airframe.py, so the two plants integrate one body. 2.0 is the thesis vehicle.
        DeclareLaunchArgument('quad_mass', default_value='2.0'),
    ]

    def _plant_node(context, *a, **k):
        plant = LaunchConfiguration('plant').perform(context).lower()
        if plant not in ('analytic', 'gazebo'):
            raise RuntimeError('plant:=%s is not one of analytic, gazebo.' % plant)
        if plant == 'analytic':
            body = _airframe().derive(float(LaunchConfiguration('quad_mass').perform(context)))
            return [Node(package=PKG, executable='uav_dynamics', name='uav_dynamics',
                         output='log',
                         parameters=[{'use_sim_time': True,
                                      'start_altitude': ParameterValue(
                                          LaunchConfiguration('start_altitude'),
                                          value_type=float),
                                      'quad_mass': body['mass'],
                                      'J_xx': body['J'][0][0],
                                      'J_yy': body['J'][1][1],
                                      'J_zz': body['J'][2][2]}])]
        return [Node(
            package=PKG, executable='gz_state_adapter', name='gz_state_adapter',
            output='log',
            parameters=[{'use_sim_time': True,
                         'unwrap_attitude': ParameterValue(
                             LaunchConfiguration('unwrap_attitude'), value_type=bool)}])]

    return LaunchDescription(args + [OpaqueFunction(function=_plant_node)])
