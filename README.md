# dvl-a50 

## Description
This repository contains a plugin for the use of the [Water Linked](https://store.waterlinked.com/product/dvl-a50/) DVL-A50 sensor in ROS2 with the advantage of making use of its new tools such as composition and Lifecycle management.

## Requirements
- [ROS2](https://docs.ros.org/en/galactic/Installation.html) - Galactic or newer
- Ubuntu 20.04 or newer

### Dependencies
- [dvl_msgs](https://github.com/paagutie/dvl_msgs)
- [JSON for Modern C++](https://github.com/nlohmann/json)


## Installation
- Clone the repositories and compile them:
```
$ source /opt/ros/galactic/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/paagutie/dvl_msgs.git
$ git clone --recurse-submodules https://github.com/paagutie/dvl-a50.git
$ cd ..
$ colcon build
```

### Usage
There are three ways to use this package. The first one uses a python script, the second one a node written in c++, for which the external library [Json](https://github.com/nlohmann/json) was used. These versions allow to run a node in a separate process with the benefits of process/fault isolation as well as easier debugging. The latest version uses [Lifecycle](https://index.ros.org/p/lifecycle/github-ros2-demos/) for node management and [composition](https://docs.ros.org/en/foxy/Tutorials/Composition.html) to increase efficiency. Thus it's possible to have more control over the TCP/IP socket configuration needed for communication. 

- First, find and set a static IP address (usually: 192.168.194.90) on your computer. 

#### Python
- To use the python script open a new terminal to run the node:
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 run dvl_a50 dvl_a50.py --ros-args -p ip_address:='192.168.194.95'
```

#### C++ 
- To use the C++ node: 
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 run dvl_a50 dvl_a50_sensor --ros-args -p dvl_ip_address:='192.168.2.95'
or
$ ros2 launch dvl_a50 dvl_a50.launch.py ip_address:='192.168.194.95'
```

## DVL Sensor Configuration

The DVL A50 sensor driver can be configured through the parameter file (see `config/dvl_a50_default.yaml`) or via command line arguments.

### DVL Sensor Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dvl_ip_address` | string | `192.168.194.95` | IP address of the DVL sensor |
| `velocity_frame_id` | string | `dvl_A50/velocity_link` | Frame ID for velocity messages |
| `position_frame_id` | string | `dvl_A50/position_link` | Frame ID for position/dead-reckoning messages |
| `use_enu` | bool | `false` | Use ENU (Z-up) coordinate frame instead of NED (Z-down) for velocity data |

### Coordinate Frame Convention

By default (`use_enu: false`), the DVL publishes velocity data in the NED (North-East-Down) coordinate frame, which is the native DVL convention with Z-axis pointing downward. 

When `use_enu: true`, the driver automatically transforms velocity data to the ENU (East-North-Up) coordinate frame with Z-axis pointing upward, which is the standard ROS convention per [REP-103](https://www.ros.org/reps/rep-0103.html). The transformation applied is:
- X_enu = Y_ned
- Y_enu = X_ned  
- Z_enu = -Z_ned

The covariance matrix is also transformed accordingly.

**Note:** Setting `use_enu: false` (default) maintains backward compatibility and avoids confusion for users familiar with the DVL's native NED frame. The DVL Navigation Interface can still handle the Z-up/Z-down conversion through TF transforms if needed.


## DVL Navigation Interface

The DVL Navigation Interface is a bridge node that converts DVL sensor data into standard ROS2 navigation messages. It processes both velocity and dead-reckoning position data from the DVL and publishes them in formats compatible with navigation stacks like `robot_localization` and `nav2`.

### What It Does

The `dvl_nav_interface` node performs the following tasks:

1. **Velocity Conversion**: Subscribes to raw DVL velocity data (`dvl/data`) and converts it to `TwistWithCovarianceStamped` messages with proper covariance matrices for sensor fusion
2. **Odometry Conversion**: Subscribes to DVL dead-reckoned position data (`dvl/position`) and converts it to standard `Odometry` messages
3. **Frame Transformations**: Handles coordinate frame transformations between:
   - DVL sensor frame and robot base_link
   - Z-down (DVL/NED convention) and Z-up (ROS convention) coordinate systems
4. **TF Broadcasting**: Optionally publishes the odometry transform to the TF tree

### Integration with Navigation Stack

The DVL nav interface outputs are designed to integrate with ROS2 navigation packages. The published topics follow standard message types and conventions per [REP-103](https://www.ros.org/reps/rep-0103.html) and [REP-105](https://www.ros.org/reps/rep-0105.html), making them compatible with sensor fusion packages like `robot_localization` and navigation frameworks like `nav2`.

**Integration Notes:**
- The `/dvl/odom` topic provides standard `nav_msgs/Odometry` messages suitable for fusion with other odometry sources
- The `/dvl/twist` topic provides velocity information with covariance for sensor fusion algorithms
- TF transforms follow REP-105 conventions with proper `odom` → `base_link` relationships
- Compatible with the `robot_localization` package for multi-sensor fusion (EKF/UKF filters)
- Can be integrated with `nav2` and other navigation stacks that consume standard odometry messages

### Topics

#### Subscriptions
- `/dvl/data` 
- `/dvl/position` 

#### Publications
- `/dvl/twist` ([geometry_msgs/TwistWithCovarianceStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)) - Linear velocity in the configured coordinate frame (Z-down by default, or Z-up if `use_enu: true`) with 6×6 covariance matrix (only linear velocity components populated from DVL's 3×3 covariance)
- `/dvl/odom` ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)) - Full odometry message with pose and covariance (only pose information from odom frame to base link frame)
- TF transform (optional): Broadcasts `odom_frame_id` → `child_frame_id` transform

### Configuration Parameters

Configure the DVL nav interface through the parameter file (see `config/dvl_a50_default.yaml`):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_frame_id` | string | `dvl_odom` | Parent frame ID for odometry messages (Z-up convention) |
| `ned_odom_frame_id` | string | `dvl_odom_ned` | NED odometry frame ID (Z-down convention) |
| `child_frame_id` | string | `base_link` | Child frame ID (typically robot base_link) |
| `orientation_variance` | double | `0.15` | Orientation variance for covariance matrix (radians²) |
| `publish_tf` | bool | `true` | Whether to publish TF (`odom_frame_id` → `child_frame_id`) |

### Frame Transformations

The nav interface requires two static transforms, which can be updated to match physical system parameters, to be published (automatically handled by `dvl_a50_launch.py`). However, transforms can be published in other ways (ex. URDF):

1. **DVL to Base Link**: Transform from `dvl_A50/position_link` to `base_link`
   - Accounts for the physical mounting position and orientation of the DVL sensor
   - Follows [REP-105](https://www.ros.org/reps/rep-0105.html) coordinate frame conventions for mobile platforms
   
2. **Z-Up to Z-Down Frame Convention**: Transform between `dvl_odom` (Z-up) and `dvl_odom_ned` (Z-down)
   - Converts between ROS standard (Z-up, ENU) and marine/DVL convention (Z-down, NED)
   - Adheres to [REP-103](https://www.ros.org/reps/rep-0103.html) axis orientation standards

### Usage

The DVL nav interface is available in both **Python** and **C++** implementations with identical functionality.

#### Running with the Full Launch File (Recommended)

The easiest way to use the DVL nav interface is with the complete launch file that starts the sensor driver, nav interface, and required TF publishers:

```bash
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 launch dvl_a50 dvl_a50_launch.py
```

The `ip_address` command line argument will override the `dvl_ip_address` parameter if specified. It is recommended to use the param file.

This launch file will start:
- DVL A50 sensor driver (`dvl_a50_sensor`)
- DVL navigation interface (C++ version: `dvl_a50_nav`)
- Static TF publishers for required frame transformations (`dvl_link_to_base_link` and `body_zup_to_zdown`)

#### Custom Configuration File

To use a custom parameter configuration:

```bash
$ ros2 launch dvl_a50 dvl_a50_launch.py params_file:=/path/to/custom.yaml 
```

### Troubleshooting

**TF Lookup Errors:**
If you see warnings about TF lookups failing:
1. Verify that the static TF publisher nodes are running (launched automatically with `dvl_a50_launch.py`):
   - `dvl_link_to_base_link` - publishes transform between `base_link` and `dvl_A50/position_link`
   - `body_zup_to_zdown` - publishes transform between `dvl_odom` and `dvl_odom_ned`
2. Check that frame IDs in your config match those in the TF tree: `ros2 run tf2_tools view_frames` and in the message headers.
3. Ensure timestamps are synchronized

**Invalid Velocity Warnings:**
The node will log warnings if `velocity_valid` is false in DVL data - this typically means:
- DVL has insufficient bottom lock
- Sensor is out of range


#### Lifecycle management (deprecated)
ROS 2 introduces the concept of managed nodes, also called LifecycleNodes. Managed nodes contain a state machine with a set of predefined states. These states can be changed by invoking a transition id which indicates the succeeding consecutive state.

- The node must first be launched using composition. This allows multiple nodes to be executed in a single process with lower overhead and, optionally, more efficient communication (see [Intra Process Communication](https://docs.ros.org/en/foxy/Tutorials/Intra-Process-Communication.html)). The idea of using composition is to be able to make use of its advantages when integrating more than one node, which is the case of a robotic system.

```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 launch dvl_a50 dvl_composition.launch.py ip_address:='192.168.194.95'
```
- Then in a new terminal the initial options can be viewed using Lifecycle. To know the available transitions:
```
$ source /opt/ros/galactic/setup.bash
$ ros2 lifecycle list /dvl_a50_node

- configure [1]
	Start: unconfigured
	Goal: configuring
- shutdown [5]
	Start: unconfigured
	Goal: shuttingdown
```

- Now it's possible to configure the node to establish communication with the sensor via TCP/IP socket:
```
$ ros2 lifecycle set /dvl_a50_node configure
```
- To know the current transition state use:
```
$ ros2 lifecycle get /dvl_a50_node

inactive [2]
```

#### Available transitions for this node using Lifecycle management
```
$ ros2 lifecycle set /dvl_a50_node activate
$ ros2 lifecycle set /dvl_a50_node deactivate
$ ros2 lifecycle set /dvl_a50_node cleanup
$ ros2 lifecycle set /dvl_a50_node shutdown
```

## ROS2 Topics 
- `/dvl/data`
- `/dvl/position`
- `dvl/config/status`
- `dvl/command/response`
- `dvl/config/command`
#### Lifecycle management
- `/dvl_a50_node/transition_event`

