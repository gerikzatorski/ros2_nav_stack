import os
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Get paths
    slam_dir = get_package_share_path('slam_2d')

    # Create the launch configuration variables
    rviz_config = LaunchConfiguration('rviz_config')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_rviz_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('slam_2d'), 'config', 'default.rviz']),
        description='Absolute path to rviz config file')

    declare_gazebo_gui_arg = DeclareLaunchArgument(
        name='gazebo_gui',
        default_value='True',
        choices=['True', 'False'],
        description='Flag to enable Gazebo client')

    declare_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        choices=['True', 'False'],
        description='Flag to use simulation time')

    # Specify the actions
    launch_wheelybot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('wheelybot_sim'), 'launch', 'simulate.launch.py'])]),
        launch_arguments={
            'rviz_config': rviz_config,
            'gazebo_gui': gazebo_gui
        }.items())

    slam_node = Node(
        package='slam_2d',
        executable='slam_node',
        output='screen',
        # parameters=[slam_params_path]
        parameters=[PathJoinSubstitution(
            [FindPackageShare('slam_2d'), 'config', 'slam.yaml'])]
    )

    ld = LaunchDescription()

    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_gazebo_gui_arg)
    ld.add_action(declare_sim_time_arg)

    ld.add_action(launch_wheelybot_sim)
    ld.add_action(slam_node)

    return ld
