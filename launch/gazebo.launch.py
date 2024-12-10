"""
Spawn Robot Description
"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='assignment_1').find('assignment_1')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/robot_assignment_1.xacro')
    default_world_path = os.path.join(test_robot_description_share, 'worlds/assignment_1.world')
    rviz_config_path = os.path.join(test_robot_description_share, 'config/rviz.rviz')
  

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    
    aruco_node = Node(
    	package='assignment_1',
    	executable='aruco_node.py',
    	name='aruco_node'
    )

    cmd_vel_pub = Node(
        package='assignment_1',
        executable='cmd_vel_pub.py',
        name='cmd_vel_pub'
    )
    

    node_controller= Node(
        package='assignment_1',
        executable='node_controller.py',
        name='node_controller'
    )
    
    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'robot_assignment_1', '-topic', '/robot_description'],
                        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node, 
        spawn_entity,
        aruco_node,
        
        

        
      
        ExecuteProcess(
            cmd=['gazebo', '--verbose', default_world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            output='screen'),
            
        node_controller,
        cmd_vel_pub,
    ])
