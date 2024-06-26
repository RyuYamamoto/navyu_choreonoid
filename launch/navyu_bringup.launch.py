from launch_ros.actions import Node

from launch import LaunchDescription, condition
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    map_path = PathJoinSubstitution([FindPackageShare("navyu_navigation"), "map", "map.yaml"])
    rviz_config = PathJoinSubstitution([FindPackageShare("navyu_navigation"), "rviz", "navyu.rviz"])

    navyu_config_path = PathJoinSubstitution(
        [FindPackageShare("navyu_choreonoid"), "config", "navyu_params.yaml"]
    )

    emcl2_config_path = PathJoinSubstitution(
        [FindPackageShare("navyu_choreonoid"), "config", "emcl2_params.yaml"]
    )

    lifecycle_node_list = ["map_server"]

    navyu_launch_path = PathJoinSubstitution([FindPackageShare("navyu_navigation"), "launch"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("localization", default_value="true"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[{"yaml_filename": map_path}],
            ),
            Node(
                package="navyu_costmap_2d",
                executable="navyu_costmap_2d_node",
                name="navyu_global_costmap_node",
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_path_planner",
                executable="navyu_global_planner_node",
                name="navyu_global_planner_node",
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_path_tracker",
                executable="navyu_path_tracker_node",
                name="navyu_path_tracker_node",
                remappings=[("/cmd_vel", "/cmd_vel_in")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="navyu_safety_limiter",
                executable="navyu_safety_limiter_node",
                name="navyu_safety_limiter_node",
                remappings=[("/cmd_vel_out", "/cmd_vel")],
                parameters=[navyu_config_path, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": lifecycle_node_list},
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([navyu_launch_path, "/emcl2.launch.py"]),
                condition=IfCondition(LaunchConfiguration("localization")),
                launch_arguments={
                  "emcl2_config_path": emcl2_config_path
                }.items()
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
