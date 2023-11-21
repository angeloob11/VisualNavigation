import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from pathlib import Path


#Add export IGN_GAZEBO_RESOURCE_PATH=<world_path>
#if there's any error with URI try ign gazebo <world_file>

def generate_launch_description():

    nav_control_node = Node(
        package="nav_control",
        executable="nav_main",
        parameters=[{
          'use_sim_time': True
        }],
        remappings=[
          ('output_vel', '/model/costar_husky_sensor_config_1/cmd_vel_relay'),
          ('/camera_img', '/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/camera_front/image')
        ],
        output='screen'
    )


    return LaunchDescription([
        nav_control_node
    ])