from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    bias_calculator_node = Node(
        package='robot_estimation',
        executable='bias_calculator',
        name='bias_calculator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    low_pass_filter_node = Node(
        package='robot_estimation',
        executable='low_pass_filter',
        name='low_pass_filter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        bias_calculator_node,
        low_pass_filter_node,
    ])
