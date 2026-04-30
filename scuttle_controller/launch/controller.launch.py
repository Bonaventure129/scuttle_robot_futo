import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

def noisy_controller(context, *args, **kwargs):
    """
    Launches the Python-based noisy odometry calculator with exaggerated wheel parameters.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python = LaunchConfiguration("use_python")

    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_py = Node(
        package="scuttle_controller",
        executable="noisy_controller.py",
        name="noisy_odom_node",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error,
             "use_sim_time": use_sim_time}],
        condition=IfCondition(use_python),
    )

    return [noisy_controller_py]

def generate_launch_description():

    # --- 1. Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="False")

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller", 
        default_value="False")

    # Set to True by default as requested
    use_python_arg = DeclareLaunchArgument(
        "use_python", 
        default_value="True")

    # Scuttle Nominal Kinematic Parameters
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius", 
        default_value="0.041")

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation", 
        default_value="0.402")

    # Simulated Errors
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error", 
        default_value="0.005")

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error", 
        default_value="0.02")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    # --- 2. Configuration Path ---
    scuttle_controller_pkg = get_package_share_directory("scuttle_controller")
    controller_config = PathJoinSubstitution([scuttle_controller_pkg, "config", "scuttle_controllers.yaml"])

    # --- 3. Controller Spawners ---
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--param-file", controller_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scuttle_controller", "-c", "/controller_manager", "--param-file", controller_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_simple_controller),
    )

    # --- 4. Simple Controller Group (Python ONLY) ---
    simple_controller_group = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
             Node(
                 package="controller_manager",
                 executable="spawner",
                 arguments=["simple_velocity_controller", "-c", "/controller_manager", "--param-file", controller_config],
                 parameters=[{"use_sim_time": use_sim_time}],
            ),
             # Python Version ONLY (C++ version removed)
             Node(
                package="scuttle_controller",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "use_sim_time": use_sim_time}],
                condition=IfCondition(use_python),
            ),
         ]
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    # --- 5. Delayed Start ---
    delayed_start_group = TimerAction(
        period=7.0,
        actions=[
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
            simple_controller_group,
            noisy_controller_launch,
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_simple_controller_arg,
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        delayed_start_group,
    ])