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

# Package share directory
pkg_share_dir = get_package_share_directory('dvl_a50')

# Default parameter file
default_params_file = os.path.join(
    pkg_share_dir,
    'config',
    'dvl_a50_default.yaml'
)

def launch_setup(context, *args, **kwargs):

    # Resolve launch arguments
    params_file = LaunchConfiguration('params_file').perform(context)
    ip_override = LaunchConfiguration('ip_address').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)

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
        namespace=namespace,
        parameters=parameters
    )

    return [dvl_node]



def generate_launch_description():
    namespace = LaunchConfiguration('namespace')


    return launch.LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Optional YAML parameter file override'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Optional namespace for the DVL nodes'
        ),
        DeclareLaunchArgument(
            'ip_address',
            default_value='',
            description='Optional override for the DVL IP address'
        ),
        launch_ros.actions.Node(
            package='dvl_a50',
            executable='dvl_a50_nav',
            name='dvl_nav_interface',
            parameters=[LaunchConfiguration('params_file')],
            namespace=namespace,
            output='screen',
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher.py',
            name='dvl_link_to_base_link',
            namespace=namespace,
            arguments=[
                '--x', '0', 
                '--y', '0', 
                '--z', '0', 
                '--qx', '1', 
                '--qy', '0', 
                '--qz', '0', 
                '--qw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'dvl_link'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_zup_to_zdown',
            namespace=namespace,
            parameters=[LaunchConfiguration('params_file')], 
            arguments=[
                '--x', '0', 
                '--y', '0', 
                '--z', '0', 
                '--qx', '1', 
                '--qy', '0', 
                '--qz', '0', 
                '--qw', '0',
                '--frame-id', 'dvl_odom',
                '--child-frame-id', 'dvl_odom_ned'],
            output='screen',
        ),
        OpaqueFunction(function=launch_setup)
    ])
