import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the URDF file
    urdf_file_path = os.path.join(get_package_share_directory('xarm_description'), 'urdf', 'xarm6', 'xarm6.urdf')
    with open(urdf_file_path, 'r') as file:
        robot_description = file.read()

    # Define the launch description
    ld = LaunchDescription()

    # Node to publish joint states
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node to publish robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Node to run RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('xarm_description'), 'rviz', 'view_only_urdf.rviz')],
    )

    # Add all nodes to the launch description
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld

