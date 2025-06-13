# robocops_ros2 - Development workspace of team RoboCops (EPFL Robot competition 2025)

This workspace contains the packages for the Robocops team. All of our packages can be found in the `/src` directory. 

## robocops_camera

The robocops_camera package is used to process the camera detections and publish the position of the duplo in the arena using the localization. It uses internal process to verified the published camera detections and then publish the official detections on a topic.

## robocops_gazebo

The robocops_gazebo package provides Gazebo simulation support for the RoboCops robotics system. It launches the simulation environment, spawns the robot at a configurable initial pose, and bridges ROS 2 and Gazebo topics using ros_gz_bridge.

### How to use

First, ensure that you have ros2 installed, sourced and that all the dependencies are installed:
```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Then build the package:

```bash
colcon build --packages-select robocops_gazebo
source install/setup.bash
```

And start the simulator:

```bash
ros2 launch robocops_gazebo arena.launch.py
```

You can find more informations about the available arguments for this commands in the next subsection.

### Customize parameters

You can of course combine the parameters. To create your own world/robot model, please check the existing files.

#### Custom worlds

To use your own '.world' files:

1. Place the '.world' file in the 'robocops_gazebo/worlds/' directory.
2. Launch using the file name without extension:
   ```bash
   ros2 launch robocops_gazebo arena.launch.py world:=your_custom_world
   ```

#### Custom robot models

To use your own robot URDF::

1. Create a .xacro file in the robocops_gazebo/description/ directory.
2. Launch using the file name without extension:
   ```bash
   ros2 launch robocops_gazebo arena.launch.py robot_model:=your_model_name
   ```
## robocops_teleop

This package provides keyboard-based teleoperation functionality for controlling the RoboCops robot via ROS 2 topics. It publishes velocity commands (geometry_msgs/msg/TwistStamped) to the /cmd_vel topic, enabling manual control of the robot during development, debugging.

Originally based on work by Darby Lim (Willow Garage) for https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/, adapted for the RoboCops team at EPFL Robot Competition 2025.

### How to use

First, ensure that you have ros2 installed, sourced and that all the dependencies are installed:
```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Then build the package:

```bash
colcon build --packages-select robocops_teleop
source install/setup.bash
```

And start the teleoperator:

```bash
ros2 run robocops_teleop teleop_keyboard
```

### Keybinds

```
Movement:
   z    
q  s  d
   x    

z/s : Increase/decrease linear velocity
q/d : Increase/decrease angular velocity
a : Stop

GPIO toggles:
w : Toggle capture routine
x : Toggle unload routine
c : Toggle button routine
v : Toggle slope_up routine
b : Toggle slope_down routine

Emergency stop: e

CTRL-C to quit
```

### Topics

| Topic        | Type                                      | Description                                      |
|--------------|-------------------------------------------|--------------------------------------------------|
| `/cmd_vel`   | `geometry_msgs/msg/Twist` or `geometry_msgs/msg/TwistStamped` | Velocity command for robot movement |

## robocops_control

The robocops_control package provides control interfaces for the RoboCops robot, specifically for controlling the robot's wheels through ROS 2 control systems. It interfaces with the robot hardware and provides control commands for wheel speed and motion.


### How to use

First, ensure that you have ros2 installed, sourced and that all the dependencies are installed:
```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Then build the package:

```bash
colcon build --packages-select robocops_control
source install/setup.bash
```

And start the interface:

```bash
ros2 run robocops_control diffbot.launch.py
```

### Customization

Various aspects can and should be customized.

First, there's the diffbot_controllers.yaml configuration file located in bringup/config.

Next, customization can be done in the URDF and the ROS 2 control configuration within the .xacro file found in the /description directory.

## robocops_brain

This package will handle high-level decision-making and robot task coordination.

## robocops_msgs

The `robocops_msgs` package defines custom message types used across the RoboCops ROS 2 system. These messages facilitate communication between perception, decision-making, and control components of the robot.

### Message Definitions

#### `Duplo.msg`

This message represents a single Duplo block detected by the system.

```plaintext
int32 id                             # Unique ID assigned to the detected Duplo
geometry_msgs/PointStamped position # Position of the Duplo in the global frame
float64 score                        # Confidence score of the detection
int32 count                          # Number of times the Duplo was observed
```

#### `Duplo.msg`

An array of Duplo blocks, typically used to publish multiple detections in a single message.

```plaintext
Duplo[] duplos # List of detected Duplos
```

## robocops_description

The `robocops_description` package contains the full robot description for the RoboCops robot, including its physical structure, sensors, and control interfaces. This package plays a critical role in both simulation and real-world deployment by providing a centralized source of truth for the robot's configuration.

### URDF & Xacro

The robot is described using **URDF** (Unified Robot Description Format) files. To improve modularity, maintainability, and reuse, the URDF is written using **Xacro** (XML Macros). This allows us to define parameters and include macros, making it easier to scale and adjust robot features like sensors, actuators, or structural changes.

## robocops_navigation

The `robocops_navigation` package is responsible for enabling autonomous navigation capabilities on the RoboCops robot. It leverages the ROS 2 Navigation Stack (Nav2) to plan paths, localize within a map, and avoid obstacles while navigating in the arena.

This package integrates various sources of motion and position feedback to compute a reliable estimate of the robot's pose and determine how to reach a specified goal.

### Configuration

All configuration files for navigation are located in the `config/` folder.

### Localization

The robot's position is estimated using:

- **Wheel Odometry**: Provided by the encoders on the motors.
- **IMU**: Helps correct heading drift and improve pose estimation.

These two sources are fused (with kallman filter) to provide a robust estimate of the robot's location.

### 🗺 Maps

The `maps/` directory contains pre-built occupancy grid maps of the arena used for localization and navigation. These maps are essential for enabling path planning and global localization.

# Setup the PI

Here you can find the command needed to setup the PI. We decided to not create a Docker container for this project since we were testing everything directly on the PI.
```
# Install Ubuntu 24.04

sudo apt update
sudo apt upgrade

sudo apt install git-all

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-ros-base

ssh-keygen

sudo apt update && sudo apt install ros-dev-tools

sudo apt install ros-jazzy-depthai-ros ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-navigation2 ros-jazzy-nav2-bringup

sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-foxy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```