import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")

    # --- CONFIG FILES ---
    #lidar_config = os.path.join(get_package_share_directory("scuttle_bringup"), "config", "rplidar_a1.yaml")
    camera_config = os.path.join(get_package_share_directory("scuttle_bringup"), "config", "camera.yaml") 

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false",
        description="If true, run SLAM. If false, run AMCL Localization."
    )
    
    # --- 1. Launch Hardware Interface (RSP + ros2_control_node) ---
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scuttle_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        ),
    )

    # --- 2. LiDAR Driver ---
    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("scuttle_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    # --- 3. Launch Controllers ---
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scuttle_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
            "use_sim_time": "False" # Real hardware uses system time
        }.items(),
    )
    
    # --- 4. Launch Joystick Teleop ---
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scuttle_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    # --- 5. IMU Driver ---
    imu_driver_node = Node(
        package="scuttle_firmware",
        executable="mpu6050_driver.py",
        output="screen"
    )
    
    # --- 6. Safety Stop Node ---
    # Ensure this executable name matches your setup.py entry point!
    # Usually it is 'safety_stop.py' if installed as a script.
    safety_stop = Node(
        package="scuttle_firmware", # Verify package name (previously scuttle_utils?)
        executable="safety_stop.py",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    # --- 7. Localization (AMCL) ---
    # Runs only if use_slam is FALSE
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( # <--- Added Missing Wrapper
            os.path.join(
                get_package_share_directory("scuttle_localization"),
                "launch",
                "global_localization.launch.py"
            )
        ),
        condition=UnlessCondition(use_slam),
        launch_arguments={
            "use_sim_time": "False",
            "map_name": "my_map" # CHANGE THIS to your actual map name
        }.items()
    )

    # --- 8. SLAM (Mapping) ---
    # Runs only if use_slam is TRUE
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( # <--- Added Missing Wrapper
            os.path.join(
                get_package_share_directory("scuttle_mapping"),
                "launch",
                "slam.launch.py"
            )
        ),
        condition=IfCondition(use_slam),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    # --- 9. USB Camera Driver (M720) ---
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        #namespace='camera',
        output='screen',
        parameters=[camera_config]
    )

    return LaunchDescription([
        #use_slam_arg,
        #hardware_interface,
        #laser_driver,
        #controller,
        #joystick,
        #imu_driver_node,
        #safety_stop,
        #localization,
        #slam
        camera_node
    ])