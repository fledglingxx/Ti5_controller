import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def load_urdf(pkg_name,urdf_file):
    pkg_path = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_path, 'urdf', urdf_file)
    with open(urdf_path, 'r') as f:
    	return f.read()


def generate_launch_description():
    package_name = 'arms_description'

    robot_description = load_urdf('arms_description', 'T170_ARMS.urdf')


    arms = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name), 'launch', 'display.launch.py'
            )]),launch_arguments={'namespace': 'arms', 'use_sim_time': 'true'}.items()
    )

    #robot_controllers = PathJoinSubstitution([FindPackageShare("Ti5_arms_controller"), 'config', 'ti5_controller.yaml'])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description  ,robot_controllers],
        output='screen'
    )
    
    

    

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['Ti5_controller', "-c", '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        arms,
        control_node,
        joint_broad_spawner,
        robot_controller_spawner,
    ])  
  