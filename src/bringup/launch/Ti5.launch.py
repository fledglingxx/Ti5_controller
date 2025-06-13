import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'arms_description'

    arms = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name), 'launch', 'display.launch.py'
            )]),launch_arguments={'namespace': 'arms', 'use_sim_time': 'true'}.items()
    )

    Ti5ArmsController_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['ti5_arms_controller', '--controller-manager', '/controller_manager'],
    parameters=[os.path.join(get_package_share_directory('Ti5_controller'), 'config', 'ti5_arms_controller.yaml')],
    output='screen'
)

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Ti5_controller = Node(
    #     package='Ti5_controller',
    #     executable='Ti5_controller_node',
    #     output='screen'
    # )

    return LaunchDescription([
        arms,
        joint_broad_spawner,
        Ti5ArmsController_spawner,
        #Ti5_controller,
    ])