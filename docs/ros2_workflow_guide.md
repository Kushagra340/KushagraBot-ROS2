# KushagraBot ROS2 Workflow Guide

This guide explains how your ROS2 packages work, the structure of your `kushagrabot_ws` workspace, and how to develop and test your robot's high-level logic in Ubuntu (WSL2).

## 1. Understanding Your Workspace

In ROS2, a **workspace** is a directory where you build and install multiple ROS2 packages. 
Your workspace is located at `C:\Users\kush2.KUSHAGRA\OneDrive\Desktop\ROBOT FILES\kushagrabot_ws`.

Inside your workspace, you have a `src` folder containing your package:
- **`kushagrabot_description`**: This package contains your URDF (the robot's physical layout), 3D meshes, RViz configurations, and Launch files (scripts that start multiple nodes at once).

*Note: Your ESP32 C++ code (`kushagrabot_firmware`) is entirely separate. It does not use the ROS2 build system (colcon) and is built using PlatformIO.*

## 2. Compiling the Workspace (Colcon Build)

Whenever you edit a file inside your ROS2 package (like changing the URDF or adding a new Python script), you need to build the workspace so ROS2 knows the files have changed.

1. Open your WSL2 Terminal.
2. Navigate to the ROOT of the workspace (never build from inside the `src` folder):
   ```bash
   cd ~/OneDrive/Desktop/"ROBOT FILES"/kushagrabot_ws
   ```
3. Run the build command:
   ```bash
   colcon build --symlink-install
   ```
   *(The `--symlink-install` flag is a neat trick. It means if you edit a Python file or a URDF file, you don't need to re-run `colcon build` to see the changes. They update instantly!)*

## 3. Sourcing the Environment

ROS2 needs to know where your compiled packages are. You have to "source" the environment **every time you open a new terminal window**.

```bash
# First, source the main ROS2 Humble installation
source /opt/ros/humble/setup.bash

# Second, source your custom workspace installation
source ~/OneDrive/Desktop/"ROBOT FILES"/kushagrabot_ws/install/setup.bash
```

*Pro-tip: You can add these two lines to the bottom of your `~/.bashrc` file so they run automatically every time you open a terminal!*

## 4. Useful ROS2 Commands for Your Robot

### Launch Files (`ros2 launch`)
Instead of running nodes one by one, you use launch files to start an entire system.
```bash
ros2 launch kushagrabot_description gazebo.launch.py
```
This single command starts the robot state publisher (which parses your URDF), opens Gazebo, and spawns the robot.

### Node Execution (`ros2 run`)
Used to run a specific executable from a package.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
This runs the keyboard controller which publishes to the `/cmd_vel` topic.

### Topic Inspection (`ros2 topic`)
Crucial for debugging the connection between your ESP32 and ROS2.
- **`ros2 topic list`**: Shows all active topics (you should see `/left_ticks`, `/cmd_vel`, etc.).
- **`ros2 topic echo /left_ticks`**: Prints the live data streaming from your ESP32's left encoder.
- **`ros2 topic hz /left_ticks`**: Tells you how fast (in Hz) the ESP32 is publishing data. We set it to 10Hz in your firmware.

### Viewing the TF Tree
ROS2 uses "Transforms" (TF) to know where every part of your robot is relative to the `base_link`.
```bash
ros2 run tf2_tools view_frames
```
This generates a PDF file showing how your wheels, ToF sensors, and castor wheel are mathematically linked together!
