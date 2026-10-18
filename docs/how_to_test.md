# KushagraBot: How to Flash and Test Your New Code!

Hey! Here is a super simple, step-by-step guide to get your new ROS 2 Control architecture running on your physical robot. Just follow these steps one by one!

## Step 1: Build & Flash the ESP32 (The Robot's Brain)
Because `micro-ROS` uses complex Linux scripts to build its C-library, **you cannot click the Build button in Windows VS Code.** It will throw an error saying `'.' is not recognized`. 

We must build it inside WSL, and then upload it!

1. Open your **Ubuntu / WSL Terminal**.
2. Run this script to compile the firmware:
   ```bash
   cd /mnt/c/Users/kush2.KUSHAGRA/OneDrive/Desktop/ROBOT_FILES
   ./build_firmware_fix.sh
   ```
3. Wait for it to say **SUCCESS**.
4. Now, plug your ESP32 into your laptop using the USB cable.
5. Open a **Windows Command Prompt** (not WSL!) and run:
   ```cmd
   cd C:\Users\kush2.KUSHAGRA\OneDrive\Desktop\ROBOT_FILES\kushagrabot_firmware
   C:\Users\kush2.KUSHAGRA\.platformio\penv\Scripts\pio.exe run -t nobuild -t upload
   ```
   *(This tells PlatformIO to take the `.bin` file we just made in WSL, and upload it without trying to rebuild it).*

## Step 2: Build the PC Code (The Laptop's Brain)
We wrote a new C++ Hardware Interface and Python GUI on your laptop in WSL. We need to compile them!

1. Open your **Ubuntu / WSL Terminal**.
2. Go to your workspace:
   ```bash
   cd ~/OneDrive/Desktop/ROBOT_FILES/kushagrabot_ws
   ```
3. Compile everything:
   ```bash
   colcon build
   ```
4. Wait for it to finish. (It shouldn't take long).

## Step 3: Turn on the Robot
1. Unplug the ESP32 from your computer.
2. Plug the ESP32 into the robot's battery/power bank.
3. Wait about 5-10 seconds for it to connect to your Wi-Fi (`Airtel_Siddhatva`).

## Step 4: Launch the Micro-ROS Agent
The agent is the bridge that lets your ESP32 talk to your laptop over Wi-Fi.

1. Open a **NEW** Ubuntu / WSL Terminal.
2. Run the Docker agent:
   ```bash
   sudo docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
   ```
3. You should see text saying the agent has started and a client (your ESP32) has connected!

## Step 5: Launch the ROS 2 Controller
Now we start the professional `diff_drive_controller` that does all the math.

1. Open a **NEW** Ubuntu / WSL Terminal.
2. Launch the hardware node:
   ```bash
   ros2 launch kushagrabot_description hardware.launch.py
   ```
3. You should see logs saying the controller manager and diff_drive_controller are active!

## Step 6: Start the Precision GUI!
Time to draw the shapes!

1. Open a **NEW** Ubuntu / WSL Terminal.
2. Run the Python GUI script:
   ```bash
   ros2 run kushagrabot_description shape_controller.py
   ```
4. A window will pop up!
5. Put your robot on the floor in an open space.
6. Click **Square** and watch it go! The GUI will show you the exact angle (Yaw) it is currently facing as it drives.

## Troubleshooting
- **Robot doesn't move:** Check the Docker terminal. Did the ESP32 connect? If not, reset the ESP32 using the "EN" button.
- **GUI says "Waiting for Odometry":** This means the `hardware.launch.py` isn't running properly, or the ESP32 isn't sending encoder data. Make sure Step 4 and 5 are running without errors!
- **Robot spins endlessly:** Make sure the left and right motor wires aren't swapped on the DRV8833!
