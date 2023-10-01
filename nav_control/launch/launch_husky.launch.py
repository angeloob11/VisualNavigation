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

    pkg_name = 'nav_control'
    WN = 3
    world_file = os.path.join(get_package_share_directory(pkg_name), f'generated/test_0{WN}/segmentation_world/segmentation_world.sdf')
    world_path = os.path.join(get_package_share_directory(pkg_name), f'generated/test_0{WN}/segmentation_world')
    print(f'export IGN_GAZEBO_RESOURCE_PATH=\"{world_path}\"')

    gz_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[
                                                EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                                                                    default_value=''),
                                                                    world_path])

    #Incluyo gazebo  con el mundo

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), 
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    #Spawneo al robot
    
    #gz ros Bridge


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/costar_husky_sensor_config_1/cmd_vel_relay@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/costar_husky_sensor_config_1/odometry@gz.msgs.Odometry',
                   '/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/world/field/model/costar_husky_sensor_config_1/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        parameters=[{'qos_overrides./model/my_robot.subscriber.reliability': 'reliable'}],
        output='screen'
    )

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
        gz_resource_path,
        gazebo,
        bridge,
        nav_control_node
    ])