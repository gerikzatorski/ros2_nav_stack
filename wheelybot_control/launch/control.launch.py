from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Specify the actions
    diff_drive_controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['diff_drive_controller'])

    joint_state_broadcaster_spawner_node = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster'])
    # Build launch description
    ld = LaunchDescription()
    ld.add_action(diff_drive_controller_spawner_node)
    ld.add_action(joint_state_broadcaster_spawner_node)
    return ld
