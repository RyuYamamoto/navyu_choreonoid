import os
import launch_ros

from launch import LaunchDescription, condition
from launch.substitutions import Command
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  use_sim_time = LaunchConfiguration("use_sim_time", default="true")

  project_path = PathJoinSubstitution([
    FindPackageShare("navyu_choreonoid"),
    "project",
    "willow_garage.cnoid"
  ])

  rviz_config = PathJoinSubstitution([
    FindPackageShare("navyu_choreonoid"),
    "rviz",
    "navyu_choreonoid.rviz"
  ])

  urdf_file = os.path.join(
    get_package_share_directory("navyu_simulator"), "urdf", "sample_robot.urdf"
  )

  robot_description = launch_ros.descriptions.ParameterValue(
    Command(["xacro", " ", urdf_file]), value_type=str
  )

  return LaunchDescription([
    DeclareLaunchArgument("use_sim_time", default_value="true"),
    DeclareLaunchArgument("use_rviz", default_value="false"),
    DeclareLaunchArgument("project_path", default_value="project_path"),

    Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}, {"use_sim_time": use_sim_time}],
    ),

    Node(
      package="topic_tools",
      executable="relay",
      arguments=["/sample_robot/hokuyo_link/scan", "scan"]
    ),

    Node(
      package="choreonoid_ros",
      executable="choreonoid",
      arguments=["--start-simulation", project_path]
    ),
    Node(
      package="rviz2",
      executable="rviz2",
      name="rviz2",
      arguments=["-d", rviz_config],
      condition=IfCondition(LaunchConfiguration("use_rviz")),
    ),
  ])
