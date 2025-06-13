# Robocops Software Stack

As explained in the report, we divided our software architecture into **three main components**, each responsible for a specific part of the system. These components are organized into three subfolders (you can find more informatios in each of the 3 readmes in the subfolder):

---

## 📟 `robocops_arduino`

This folder contains the **firmware code for the Arduino-based hardware controller**. It is responsible for:

- Driving the motors (Maxon motors, brushes, etc.)
- Reading and encoding sensor data (encoders, ultrasonic sensors, etc.)
- Communicating over serial using a custom protocol
- Interpreting control commands from the onboard Raspberry PI via USB

The Arduino listens for 5-byte commands and responds with an 11-byte state report, including motor encoders, control flags, duplos count, and ultrasonic distance. More details about the protocol can be found in the documentation within this folder. 

---

## 🧠 `robocops_ros2`

This folder contains the **ROS 2 packages** that coordinate high-level behavior of the robot. The ROS 2 system is built with modularity in mind, allowing components to be debugged and extended independently.

---

## 🎯 `robocops_computervision`

This folder contains the **research and implementation of the computer vision system**. It includes:

- Object detection and classification models for identifying duplos
- Experimentation notebooks and training scripts

All the vision work here is packaged into a ROS 2 node in the `robocops_camera` package (in the `robocops_ros2` folder), where it publishes detection results used by the robot for decision-making.
