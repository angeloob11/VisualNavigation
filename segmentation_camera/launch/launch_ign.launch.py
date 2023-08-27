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

    pkg_name = 'segmentation_camera'
    WN = 3
    world_file = os.path.join(get_package_share_directory(pkg_name), f'generated/test_0{WN}/segmentation_world/segmentation_world.sdf')
    world_path = os.path.join(get_package_share_directory(pkg_name), f'generated/test_0{WN}/segmentation_world')

    gz_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[
                                                EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                                                                    default_value=''),
                                                                    world_path])

    #Incluyo el anterior launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        os.path.join(
        get_package_share_directory(pkg_name), 'launch/rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    #Incluyo gazebo  con el mundo

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), 
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    #Rviz 2 configuration
    """
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(get_package_share_directory(pkg_name), )
        ]
    )
    """

    #Spawneo al robot

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_robot',
                                   '-x','-1.0',
                                   '-y','-1.0'],
                        output = 'screen')
    
    #gz ros Bridge

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/my_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/my_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        parameters=[{'qos_overrides./model/my_robot.subscriber.reliability': 'reliable'}],
        output='screen'
    )
    
    return LaunchDescription([
        gz_resource_path,
        rsp,
        gazebo,
        bridge,
        spawn_entity,
    ])