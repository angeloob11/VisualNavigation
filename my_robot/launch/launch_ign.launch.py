import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

world_file = '$HOME/tdg_ws/src/my_robot/generated/test_03/segmentation_world/segmentation_world.sdf'

def generate_launch_description():

    pkg_name = 'my_robot'

    #Incluyo el anterior launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        os.path.join(
        get_package_share_directory(pkg_name), 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    #Incluyo gazebo  on el mundo

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
        rsp,
        gazebo,
        bridge,
        spawn_entity,
    ])