"""
ROS 2 launch file for the DVL A50 driver.

Supported usage:
----------------
# Use default parameters
ros2 launch dvl_a50 driver.launch.py

# Override IP address only
ros2 launch dvl_a50 driver.launch.py ip_address:=192.168.1.42

# Use a custom parameter file
ros2 launch dvl_a50 driver.launch.py params_file:=/path/to/custom.yaml

# Override both
ros2 launch dvl_a50 driver.launch.py \
  params_file:=custom.yaml \
  ip_address:=192.168.2.95
"""

import os

import launch
import launch_ros.actions

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    # Package share directory
    pkg_share_dir = get_package_share_directory('dvl_a50')

    # Default parameter file
    default_params_file = os.path.join(
        pkg_share_dir,
        'config',
        'dvl_a50_default.yaml'
    )

    # Resolve launch arguments
    params_file = LaunchConfiguration('params_file').perform(context)
    ip_override = LaunchConfiguration('ip_address').perform(context)

    # Select parameter file
    if params_file == '':
        params_file = default_params_file

    # Parameter list (order matters — later overrides earlier)
    parameters = [params_file]

    # Optional IP override
    if ip_override != '':
        parameters.append({
            'dvl_ip_address': ip_override
        })

    dvl_node = launch_ros.actions.Node(
        package='dvl_a50',
        executable='dvl_a50_sensor',
        output='screen',
        parameters=parameters
    )

    return [dvl_node]



def generate_launch_description():

    return launch.LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Optional YAML parameter file override'
        ),

        DeclareLaunchArgument(
            'ip_address',
            default_value='',
            description='Optional override for the DVL IP address'
        ),
        OpaqueFunction(function=launch_setup)
    ])
