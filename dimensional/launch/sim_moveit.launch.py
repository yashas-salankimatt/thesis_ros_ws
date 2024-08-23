import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # start ROS TCP endpoint
    ros_tcp_endpoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_tcp_endpoint'), 'launch', 'endpoint.py'])),
    )

    hw_ns = LaunchConfiguration('hw_ns', default='xarm')

    # robot moveit fake launch
    robot_moveit_fake_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_fake.launch.py'])),
        launch_arguments={
            'dof': '6',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
        }.items(),
    )

    # sim_cam_tf_pub node
    sim_cam_tf_pub_node = Node(
        package='dimensional',
        executable='sim_cam_tf_pub',
        output='screen',
    )

    # # point_cloud_creator node
    # point_cloud_creator_node = Node(
    #     package='dimensional',
    #     executable='point_cloud_creator',
    #     output='screen',
    # )

    return LaunchDescription([
        ros_tcp_endpoint_launch,
        robot_moveit_fake_launch,
        sim_cam_tf_pub_node,
        # point_cloud_creator_node,
    ])