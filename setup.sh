#!/bin/bash

# SCUTTLE Robot Dependency Installer
# Usage: ./setup_dependencies.sh

echo "🐢 SCUTTLE Robot Setup Script"
echo "----------------------------"
echo "1. PC / Laptop (Full Simulation Support)"
echo "2. Raspberry Pi (Hardware Only - Skips Gazebo)"
echo "----------------------------"
read -p "Select your platform (1 or 2): " platform

echo "Updating rosdep..."
sudo apt update
rosdep update

if [ "$platform" = "1" ]; then
    echo "📦 Installing PC dependencies (including Gazebo)..."
    rosdep install --from-paths src --ignore-src -r -y
    
    echo "📦 Installing Python AI libraries..."
    pip3 install ultralytics vosk pyaudio pyttsx3 pyserial smbus

elif [ "$platform" = "2" ]; then
    echo "📦 Installing Raspberry Pi dependencies (Skipping Gazebo)..."
    rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys "ros_gz_sim ros_gz_bridge gazebo_ros gazebo_plugins"
    
    echo "📦 Installing Python Hardware libraries..."
    pip3 install pyserial smbus
    
    echo "🔧 Setting up Serial Permissions..."
    sudo usermod -aG dialout $USER
    sudo usermod -aG input $USER
    sudo usermod -aG video $USER
    echo "⚠️  NOTE: Please REBOOT your Pi to apply permission changes!"

else
    echo "Invalid selection. Exiting."
    exit 1
fi

echo "✅ Setup Complete! You can now build with: colcon build --symlink-install"
