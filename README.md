# Development workspace of team RoboCops (EPFL Robot competition 2025)

This workspace contains the packages for the Robocops team. All of our packages can be found in the `/src` directory.

## robocops_duplos

The robocops_duplos package is used to process the camera detections and publish the position of the duplo in the arena using the localization. It uses internal process to verified the published camera detections.

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
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space or s : force stop

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

## robocops_camera

The `robocops_camera` package is responsible for setting up and operating OAK-D Lite camera for object detection and other camera-related tasks in the Robocops robotics system.

### How to use

#### Step 1: Prepare your setup

Before starting the package, ensure that the DepthAI camera is properly connected to your system. 

1. **Plug in the Camera**: Connect the camera via USB to your machine.
2. **Set Up the Environment**: Make sure you have a valid ROS 2 environment set up, with the necessary dependencies installed (depthai & ROS2 Jazzy).

#### Step 2: Build the package

First, install the dependencies and build the workspace:

```bash
# Install ROS2 dependencies (for the 'jazzy' distro)
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

# Build the workspace
colcon build

# Source the setup files
source install/setup.bash
```

#### Step 3: Launch the package

Once everything is set up, you can launch the robocops_camera package, which includes object detection and visualization tools. There are different configurations based on your needs:

1. Basic object detection (compet mode)

This will start the object detection system with the default parameters (without display):
```bash
ros2 launch robocops_camera duplo_detector.py
```

2. Object detection with display (dev mode)

If you want to visualize the camera feed along with the bounding boxes and other detections, use this command:
```bash
ros2 launch robocops_camera duplo_detector.py with_display:=true
```

#### Step 4: Customize parameters
You can customize various parameters of the object detection system. Some of the configurable options are:

- nn_name: Path to the neural network model file (e.g., yolo11n_3.blob).
- rgb_resolution_str: The resolution of the RGB camera (e.g., 1080p, 720p).
- with_display: Whether to display the camera feed with bounding boxes (true or false).
- resource_base_folder: The path to the resources file (e.g. /home/USER/dev_ws/src/resources)

For example, to specify a custom neural network file and resolution:
```bash
ros2 launch robocops_camera duplo_detector.py nn_name:=/path/to/custom_model.blob rgb_resolution_str:=720p
```

### Topics

When the `robocops_camera` package is launched, the following ROS 2 topics are created for interaction:

1. **/camera/raw_rgb** (only if display is enabled) 
   Type: `sensor_msgs/msg/Image`  
   Description: The raw RGB camera feed from the OAK-D Lite camera. This topic provides real-time image data captured by the RGB camera.

2. **/camera/depth** (only if display is enabled) 
   Type: `sensor_msgs/msg/Image`  
   Description: Depth data from the OAK-D Lite's stereo depth camera. This topic provides the depth information corresponding to the RGB feed, which can be used for spatial understanding.

3. **/camera/detections**  
   Type: `depthai_ros_msgs/msg/SpatialDetectionArray`  
   Description: The spatial detections output by the YOLO-based object detection network. This topic includes the detected objects' bounding boxes, confidence scores, and spatial coordinates in the camera's frame.


# Setup the PI

Install Ubuntu 24.04

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

sudo apt install ros-jazzy-depthai-ros

sudo apt install ros-jazzy-navigation2

sudo apt install ros-jazzy-nav2-bringup

sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
