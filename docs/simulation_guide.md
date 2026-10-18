# KushagraBot Simulation Guide (Gazebo & RViz)

Welcome to the simulation environment for KushagraBot! This guide will walk you through exactly how to boot up your robot in a virtual world, drive it around, and troubleshoot issues.

## 1. Getting Started: Booting the Simulation

Follow these steps exactly after turning on your laptop and opening WSL2 (Ubuntu):

1. **Open your Terminal (WSL2).**
2. **Navigate to your workspace:**
   ```bash
   cd ~/OneDrive/Desktop/ROBOT_FILES/kushagrabot_ws
   ```
3. **Build the workspace** (You only need to do this if you changed the URDF/Xacro files):
   ```bash
   colcon build --symlink-install
   ```
4. **Source the ROS2 environment:**
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```
5. **Launch the Gazebo Simulator:**
   ```bash
   ros2 launch kushagrabot_description gazebo.launch.py
   ```
   *What happens:* Gazebo will open, showing a `playground` world with your robot and three colored boxes (Red, Green, Blue) acting as obstacles. The two infrared ToF beams will be visible emerging from the front of your robot.

6. **Open a SECOND Terminal** to launch RViz (to see what the robot "sees"):
   ```bash
   cd ~/OneDrive/Desktop/"ROBOT FILES"/kushagrabot_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   rviz2 -d src/kushagrabot_description/rviz/urdf_config.rviz
   ```

## 2. Driving the Robot via Keyboard

To control the robot in the simulation, open a **THIRD Terminal** and run the standard ROS2 teleop node:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- **Controls:**
  - `i` : Move forward
  - `,` : Move backward
  - `j` : Turn left
  - `l` : Turn right
  - `k` : Stop
  - `q` / `z` : Increase/Decrease max speeds

*Keep this terminal active/selected while pressing the keys.*

## 3. Key Parameters & What They Do

If you want to tweak how the robot behaves in the simulation, you can edit `kushagrabot_gazebo.xacro`:

- **`<wheel_separation>0.136</wheel_separation>`**: The distance between the left and right wheels in meters. 
  - *If you change this:* The robot will turn faster or slower in the simulation because the math for differential drive changes. If it doesn't match your physical robot, your simulation odometry will be wrong.
- **`<wheel_diameter>0.034</wheel_diameter>`**: The diameter of your wheels (34mm).
  - *If you change this:* It changes how far the robot thinks it travels per wheel rotation.
- **`<update_rate>30</update_rate>` (Diff Drive Plugin)**: How often (Hz) the plugin publishes odometry (`/odom`).
  - *If you change this:* A higher number provides smoother tracking but uses more CPU. 30Hz is standard.
- **`<max_wheel_torque>20</max_wheel_torque>`**: How much power the simulated motors have.
  - *If you change this:* If too low, the robot won't have enough power to move its own weight (`body_mass`).
- **`<mu1>1.0</mu1>` and `<mu2>1.0</mu2>`**: Friction coefficients for the wheels.
  - *If you change this:* Setting this lower will cause the wheels to slip on the Gazebo floor like ice. The castor wheel has this set to `0.001` so it slides frictionlessly instead of getting stuck.

## 4. Troubleshooting Simulation Issues

- **Issue:** The robot starts jumping into the air or vibrating violently when Gazebo opens.
  - *Cause:* Physics collision geometries are clipping into each other.
  - *Fix:* Ensure `wheel_z_offset` and `castor_z_offset` in `kushagrabot.urdf.xacro` keep the `base_link` box from touching the ground.

- **Issue:** The robot won't move when I press keys in the `teleop_twist_keyboard` terminal.
  - *Cause:* Gazebo is paused, or the terminal running teleop is not selected.
  - *Fix:* Press the "Play" button in the bottom left of the Gazebo window. Click inside the teleop terminal before pressing keys.

- **Issue:** The robot turns left when I press right (or vice versa).
  - *Cause:* The `teleop` node assumes positive angular Z means turning left (counter-clockwise). If your robot turns right, the wheel joints or differential drive plugin mappings are flipped.
  - *Fix:* Swap the `<left_joint>` and `<right_joint>` in `kushagrabot_gazebo.xacro`.

---
*Created by Antigravity IDE.*
