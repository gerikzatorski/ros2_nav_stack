from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Create the launch configuration variables
    model = LaunchConfiguration('model')

    # Declare the launch arguments
    declare_model_arg = DeclareLaunchArgument(
        name='model',
        default_value=PathJoinSubstitution(
            [FindPackageShare('wheelybot_description'), 'urdf', 'wheelybot.xacro']),
        description='Absolute path to robot urdf file')

    # Specify the actions
    robot_description_content = ParameterValue(
        Command(['xacro ', model]),
        value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}])

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher')

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(declare_model_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    return ld
