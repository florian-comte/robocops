# Development workspace of team RoboCops (EPFL Robot competition 2025)

This workspace contains the packages for the Robocops team. All of our packages can be found in the `/src` directory.

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

### Created topics

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
