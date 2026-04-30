#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ROS 2 Launch File for Scuttle Robot Visualization
# Purpose: Launches the necessary nodes (robot_state_publisher, joint_state_publisher_gui, RViz2) 
#          to visualize the Scuttle robot's URDF model in a simulated environment.
# Package: scuttle_description
# ROS 2 Distro: Jazzy (or later)

import os
# Imports the ROS 2 package indexing utility to dynamically locate package share directories.
from ament_index_python.packages import get_package_share_directory

# Core Launch System Imports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# Imports for command execution and substitution within the launch file.
from launch.substitutions import Command, LaunchConfiguration

# ROS Node Imports
from launch_ros.actions import Node
# Wrapper class to define parameters whose values are derived from substitutions (like Xacro output).
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Generates the LaunchDescription object containing all visualization nodes and configuration.
    
    Returns:
        LaunchDescription: The configuration for the ROS 2 launch system.
    """

    # --- 1. RESOURCE DIRECTORY RESOLUTION ---
    # Dynamically retrieves the installation path of the 'scuttle_description' package.
    # This is a ROS 2 best practice for portability across different build/install environments.
    scuttle_description_dir = get_package_share_directory("scuttle_description") 

    # --- 2. LAUNCH ARGUMENT DEFINITION ---
    # Defines an optional command-line argument 'model' to allow users to specify an alternative 
    # URDF/Xacro file path at launch time (e.g., ros2 launch ... model:=/path/to/other.urdf.xacro).
    model_arg = DeclareLaunchArgument(
        name="model", 
        # Sets the default model to the primary Scuttle robot Xacro file.
        default_value=os.path.join(scuttle_description_dir, "urdf", "scuttle.urdf.xacro"),
        description="Absolute path to the robot's URDF or XACRO definition file."
    )

    # --- 3. ROBOT DESCRIPTION PARAMETER PROCESSING ---
    # Prepares the 'robot_description' parameter, which holds the robot's model definition.
    # The 'Command' utility executes the 'xacro' command on the file specified by 'model_arg'.
    # This processes the Xacro file (including macros and includes) into a single, valid URDF XML string.
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # --- 4. ROBOT STATE PUBLISHER NODE ---
    # Reads the 'robot_description' parameter and joint positions (published by the J-S-P-GUI)
    # to calculate and publish the robot's kinematic chain transforms to the /tf topic. 
    # This is critical for RViz2 to correctly position the robot links relative to each other.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # --- 5. JOINT STATE PUBLISHER GUI NODE ---
    # Provides an interactive graphical interface (sliders) to manually control the values of all 
    # non-fixed joints defined in the URDF. It publishes these values to the /joint_states topic.
    # Essential for verifying the URDF kinematics and ranges of motion during development.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # --- 6. RVIZ2 VISUALIZATION NODE ---
    # Launches the main ROS 2 visualization tool.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # Loads a pre-configured RViz state file. This ensures the correct displays (RobotModel, TF)
        # and viewpoints are active upon startup for an optimal visualization experience.
        arguments=["-d", os.path.join(scuttle_description_dir, "rviz", "display.rviz")],
    )

    # --- 7. LAUNCH DESCRIPTION ASSEMBLY ---
    # Groups all declared arguments and nodes for execution by the launch system.
    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])