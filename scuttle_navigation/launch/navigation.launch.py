import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    scuttle_navigation_pkg = get_package_share_directory("scuttle_navigation")

    # Define the list of nodes for the lifecycle manager to handle
    # Order matters slightly: usually servers first, then navigator
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        #"smoother_server",
        "recoveries_server",
        "bt_navigator"
    ]

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    # 1. Controller Server
    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            os.path.join(scuttle_navigation_pkg, "config", "controller_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # 2. Planner Server
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(scuttle_navigation_pkg, "config", "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    # 3. Smoother Server
   # nav2_smoother_server = Node(
    #    package="nav2_smoother",
    #    executable="smoother_server",
    #    name="smoother_server",
    #    output="screen",
    #    parameters=[
    #       os.path.join(scuttle_navigation_pkg, "config", "smoother_server.yaml"),
    #        {"use_sim_time": use_sim_time}
    #   ],
    #)

    # 4. Recoveries Server (Spin, BackUp, Wait)
    # You will need a recoveries_server.yaml, or reuse valid params
    nav2_recoveries_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="recoveries_server",
        output="screen",
        parameters=[
            os.path.join(scuttle_navigation_pkg, "config", "recoveries_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    # 5. BT Navigator (The Coordinator)
    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(scuttle_navigation_pkg, "config", "bt_navigator.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    # 6. Lifecycle Manager
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_controller_server,
        nav2_planner_server,
        #nav2_smoother_server,
        nav2_recoveries_server,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
    ])