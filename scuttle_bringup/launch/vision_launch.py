import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Locate the customized camera.yaml configuration file
    camera_config = os.path.join(
        get_package_share_directory("scuttle_bringup"), 
        "config", 
        "camera.yaml"
    )

    # 1. The USB Camera Driver Node (Using your snippet)
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[camera_config]
    )

    # 2. Your Custom YOLO Detection Node
    yolo_node = Node(
        package='scuttle_bringup', 
        executable='yolo_detector.py',
        name='yolo_detector',
        output='screen'
    )

    # 3. New Object Tracker Node
    tracker_node = Node(
        package='scuttle_bringup', 
        executable='object_tracker.py', 
        name='object_tracker',
        output='screen'
    )
    # Launch both nodes simultaneously
    return LaunchDescription([
        camera_node,
        #yolo_node,
        tracker_node,
    ])