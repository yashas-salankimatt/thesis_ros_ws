import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="xarm6", package_name="xarm_moveit_config")
        .robot_description(file_path=get_package_share_directory("xarm_moveit_config") + "/srdf/xarm.srdf.xacro")
        .trajectory_execution(file_path=get_package_share_directory("xarm_moveit_config") + "/config/xarm6/fake_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("dimensional")
            + "/config/planning_config.yaml"
        )
        .to_moveit_configs()
    )

    grasp_node = DeclareLaunchArgument(
        "grasp_node",
        default_value="grasp_pipeline_test.py",
        description="Grasp node to launch",
    )

    moveit_py_node = Node(
        name="grasp_pipeline_test",
        package="dimensional",
        executable=LaunchConfiguration("grasp_node"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([grasp_node, moveit_py_node])