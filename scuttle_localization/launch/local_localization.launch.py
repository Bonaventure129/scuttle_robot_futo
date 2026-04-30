import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    # --- 1. Arguments ---
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="True",
        description="If true, launch the Python IMU republisher.",
    )
    
    # ADD THIS ARGUMENT
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true"
    )

    use_python = LaunchConfiguration("use_python")
    use_sim_time = LaunchConfiguration("use_sim_time") # Capture the configuration

    # --- 2. Configuration Path ---
    scuttle_localization_pkg = get_package_share_directory("scuttle_localization")
    ekf_config_path = os.path.join(scuttle_localization_pkg, "config", "ekf.yaml")
    
    # --- 3. Nodes ---
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_path, 
            {'use_sim_time': use_sim_time} # <--- CRITICAL ADDITION
        ],
    )

    static_transform_publisher = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["--x", "0", "--y", "0","--z", "0.0746", 
               "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", # Identity quaternion
               "--frame-id", "base_link_ekf",
               "--child-frame-id", "imu_link_ekf"],
    )
# Add it to the return list!

    imu_republisher_py = Node(
        package="scuttle_localization",
        executable="imu_republisher.py",
        name="imu_republisher_node",
        condition=IfCondition(use_python),
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    return LaunchDescription([
        use_python_arg,
        use_sim_time_arg,
        robot_localization_node,
        imu_republisher_py,
        static_transform_publisher,

    ])