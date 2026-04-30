import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 1. Get Package Paths
    scuttle_description_dir = get_package_share_directory("scuttle_description")
    scuttle_controller_dir = get_package_share_directory("scuttle_controller")

    # 2. Process URDF (Set is_sim:=False for Real Hardware)
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(scuttle_description_dir, "urdf", "scuttle.urdf.xacro"),
                " is_sim:=False"
            ]
        ),
        value_type=str,
    )

    # 3. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # 4. Controller Manager (The Main ROS 2 Control Node)
    # This loads the C++ hardware interface plugin
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(scuttle_controller_dir, "config", "scuttle_controllers.yaml"),
        ],
    )

    # NOTE: We do NOT spawn controllers here anymore. 
    # real_robot.launch.py will call controller.launch.py to handle that.

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager,
        ]
    )