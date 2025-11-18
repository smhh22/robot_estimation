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
    controller_motion_model_node = Node(
        package='robot_estimation',
        executable='controller_motion_model',
        name='controller_motion_model',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    odom_estimator_node = Node(
        package='robot_estimation',
        executable='odom_estimator',
        name='odom_estimator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        bias_calculator_node,
        low_pass_filter_node,
        controller_motion_model_node,
        odom_estimator_node,
    ])
