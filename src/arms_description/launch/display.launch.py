from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os


def generate_launch_description():
    # 直接获取包路径字符串
    description_package_path = get_package_share_directory('arms_description')
    urdf_file = os.path.join(description_package_path, 'urdf', 'T170_ARMS.urdf')

    with open(urdf_file, 'r') as infp:
        urdf = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            description='Whether to start RViz'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])
