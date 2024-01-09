import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():

    nav_control_node = Node(
        package="nav_control",
        executable="nav_main",
        parameters=[{
          'use_sim_time': True
        }],
        remappings=[
          ('output_vel', '/model/marble_husky_sensor_config_1/cmd_vel_relay'),
          ('/camera_img', '/world/field/model/marble_husky_sensor_config_1/link/tilt_gimbal_link/sensor/camera_pan_tilt/image')
        ],
        output='screen'
    )

    img_treat_node = Node(
        package="img_treat",
        executable="img_treat_CV",
        parameters=[{
          'use_sim_time': True
        }],
        remappings=[
          ('/camera_img', '/world/field/model/marble_husky_sensor_config_1/link/tilt_gimbal_link/sensor/camera_pan_tilt/image')
        ],
        output='screen'
    )


    return LaunchDescription([
        nav_control_node,
        img_treat_node
    ])