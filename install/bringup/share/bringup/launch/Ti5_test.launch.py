# Ti5_controller/src/bringup/launch/Ti5_test.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    arms_description = get_package_share_directory('arms_description')
    moveit_config = get_package_share_directory('T170_ARMS_moveit_config')
    hardware_interface = get_package_share_directory('Ti5_hardware_interface')
    moveit_interface = get_package_share_directory('Ti5_moveit_interface')

    # 启动 robot_state_publisher + joint_state_broadcaster
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config, 'launch', 'rsp.launch.py'))
    )

    # 启动 MoveIt move_group 节点和控制器
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config, 'launch', 'move_group.launch.py'))
    )

    # 启动 RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config, 'launch', 'moveit_rviz.launch.py'))
    )

    # 启动 hardware_interface
    hardware_node = Node(
        package='Ti5_hardware_interface',
        executable='ti5_hardware_interface_node',
        name='ti5_hardware_interface',
        output='screen'
    )

    # 启动 Ti5_moveit_interface
    moveit_interface_node = Node(
        package='Ti5_moveit_interface',
        executable='Ti5_moveit_interface_node',
        name='ti5_moveit_interface',
        output='screen'
    )

    return LaunchDescription([
        rsp_launch,
        hardware_node,
        move_group_launch,
        rviz_launch,
        TimerAction(
            period=2.0,
            actions=[moveit_interface_node]
        )
    ])
