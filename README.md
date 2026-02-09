# **🐢 SCUTTLE Robot \- ROS 2 Jazzy**

A comprehensive autonomous mobile robot stack for the **SCUTTLE Platform**, built on **ROS 2 Jazzy**. This project features a full Navigation 2 stack, SLAM, YOLOv8 object tracking, and offline voice control.

## **🏗️ Architecture**

This repository is organized as a meta-package containing the following modules:

| Package | Description |
| :---- | :---- |
| **scuttle\_bringup** | Main launch files (Real/Sim) and AI nodes (YOLO, Voice, Tracking). |
| **scuttle\_description** | URDF model, meshes, and Gazebo Harmonic simulation config. |
| **scuttle\_navigation** | Nav2 configuration, Behavior Trees, and maps. |
| **scuttle\_mapping** | SLAM Toolbox configuration and custom occupancy grid tools. |
| **scuttle\_firmware** | Hardware Interface (C++) for Arduino Serial communication. |
| **scuttle\_controller** | DiffDrive control, Twist Mux, and Teleoperation (Keyboard/Joy). |

## **💻 Installation (PC / Simulation)**

Use these instructions to run the **Gazebo Simulation** on your Laptop/Desktop.

### **1\. Prerequisites**

* **OS:** Ubuntu 24.04 LTS (Noble Numbat)  
* **ROS 2:** Jazzy Jalisco ([Install Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html))

### **2\. Setup Workspace**

mkdir \-p \~/ros2\_ws/src  
cd \~/ros2\_ws/src  
git clone \[https://github.com/YOUR\_USERNAME/scuttle\_ros2.git\](https://github.com/YOUR\_USERNAME/scuttle\_ros2.git) .

### **3\. Install Dependencies**

cd \~/ros2\_ws  
sudo apt update  
rosdep update  
\# Installs everything including Gazebo and Simulation tools  
rosdep install \--from-paths src \--ignore-src \-r \-y

### **4\. Install Python Libraries (AI Features)**

pip3 install ultralytics  \# YOLOv8  
pip3 install vosk         \# Offline Voice Recognition  
pip3 install pyaudio      \# Microphone access  
pip3 install pyttsx3      \# Text-to-Speech

### **5\. Build**

colcon build \--symlink-install  
source install/setup.bash

## **🍓 Installation (Raspberry Pi \- Hardware)**

Use these instructions for the **Physical Robot**.

### **1\. Install Dependencies (Skip Simulation)**

On the Raspberry Pi, we skip the heavy simulation packages (ros\_gz\_sim, gazebo, etc.) to save space and build time.

cd \~/ros2\_ws  
rosdep update  
\# The \--skip-keys argument prevents installing Gazebo on the Pi  
rosdep install \--from-paths src \--ignore-src \-r \-y \\  
  \--skip-keys "ros\_gz\_sim ros\_gz\_bridge gazebo\_ros gazebo\_plugins"

### **2\. Hardware Permissions**

Grant your user access to the Serial ports (Arduino/Lidar) and Input devices (Joystick). **You must reboot after running this.**

sudo usermod \-aG dialout $USER  
sudo usermod \-aG input $USER  
sudo usermod \-aG video $USER

### **3\. Python Hardware Libraries**

pip3 install pyserial     \# For Arduino Communication  
pip3 install smbus        \# For I2C IMU

## **🔌 Hardware Setup**

### **Wiring Map**

| Component | Pin / Port | Notes |
| :---- | :---- | :---- |
| **Left Motor** | PWM 3, Dir 9 | Arduino Uno/Nano |
| **Right Motor** | PWM 11, Dir 10 | Arduino Uno/Nano |
| **Encoders** | I2C (SDA/SCL) | Addr: 0x40 (L), 0x41 (R) |
| **Lidar** | /dev/ttyUSB0 | RPLidar A1 |
| **Camera** | /dev/video0 | USB Webcam |
| **IMU** | I2C Bus 1 | MPU6050 (Addr 0x68) |

### **Microcontroller Firmware**

1. Open scuttle\_firmware/firmware/robot\_control.ino in the Arduino IDE.  
2. Install the **AMS\_AS5048B** library (by sosandroid) via Library Manager.  
3. Flash the code to your Arduino.

## **🚀 Usage**

### **1\. Simulation (Gazebo)**

Launch the robot in a virtual world with Navigation active:

ros2 launch scuttle\_bringup simulated\_robot.launch.py

* **RViz** will open automatically.  
* Use **"2D Pose Estimate"** to localize.  
* Use **"Nav2 Goal"** to send the robot to a target.

### **2\. Real Robot**

Launch the drivers (Lidar, Camera, Arduino, Mic) and Navigation stack:

ros2 launch scuttle\_bringup real\_robot.launch.py

### **3\. AI Features**

* **Voice Control:** The robot listens for commands.  
  * *"Forward"*, *"Back"*, *"Left"*, *"Right"*, *"Stop"*.  
* **Object Tracking:**  
  * Show a **Person** to the camera.  
  * The robot will rotate to face the person and maintain a safe distance.  
  * *(Configurable in yolo\_detector.py)*.

### **4\. Mapping (SLAM)**

To create a new map of your room:

\# Sim  
ros2 launch scuttle\_bringup simulated\_robot.launch.py use\_slam:=true

\# Real  
ros2 launch scuttle\_bringup real\_robot.launch.py use\_slam:=true

## **🛠️ Troubleshooting**

**1\. "Serial Port Permission Denied"**

* Make sure you ran the usermod commands in the Installation section and **rebooted**.  
* Check connection: ls \-l /dev/ttyUSB\*

**2\. "Voice Control not hearing me"**

* Check your microphone: arecord \-l  
* Ensure vosk-model is downloaded (The code attempts to download it, but manual download may be required if internet is slow).

**3\. "Navigation is jittery"**

* Check the scuttle\_controller/odom topic. If the odometry is spinning while the robot is still, check your Encoder wiring or PID values in robot\_control.ino.

## **📜 License**

This project is licensed under the Apache 2.0 License \- see the LICENSE file for details.