import os
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Create the launch configuration variables
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    
    # Declare the launch arguments
    declare_model_arg = DeclareLaunchArgument(
        name='model',
        default_value=PathJoinSubstitution(
            [FindPackageShare('wheelybot_sim'), 'urdf', 'wheelybot.xacro']),
        description='Absolute path to robot urdf file')

    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value=PathJoinSubstitution(
            # [FindPackageShare('wheelybot_sim'), 'worlds', 'wheely_room.sdf']),
            [FindPackageShare('wheelybot_sim'), 'worlds', 'wheely_room.sdf']),
        description='Full path to world model file to load')

    declare_rviz_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('wheelybot_sim'), 'config', 'default.rviz']),
        description='Absolute path to rviz config file')

    declare_gazebo_gui_arg = DeclareLaunchArgument(
        name='gazebo_gui',
        default_value='True',
        choices=['True', 'False'],
        description='Flag to enable Gazebo client')

    # Specify the actions
    launch_wheelybot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('wheelybot_description'), 'launch', 'description.launch.py'])]),
        launch_arguments={
            'model': model
        }.items())

    launch_wheelybot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('wheelybot_control'), 'launch', 'control.launch.py'])]))

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([
            FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]),
        launch_arguments={
            # 'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path,
            'world': world,
            'gui': gazebo_gui
        }.items())

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config])

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=['-topic', '/robot_description', '-entity', 'wheelybot', '-x', '-2.0', '-y', '-1.0'])

    # Build launch description
    ld = LaunchDescription()
    ld.add_action(declare_model_arg)
    ld.add_action(declare_world_arg)
    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_gazebo_gui_arg)
    ld.add_action(launch_wheelybot_description)
    ld.add_action(launch_wheelybot_control)
    ld.add_action(launch_gazebo)
    ld.add_action(rviz_node)
    ld.add_action(spawn_entity_node)
    return ld
