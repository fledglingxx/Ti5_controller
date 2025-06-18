# bringup/launch/Ti5_test.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = get_package_share_directory('T170_ARMS_moveit_config')
    ti5_interface = get_package_share_directory('Ti5_moveit_interface')

    # 路径配置
    controllers_yaml = os.path.join(moveit_config, 'config', 'ros2_controllers.yaml')

    # robot_state_publisher 和 joint_state_broadcaster
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config, 'launch', 'rsp.launch.py')
        )
    )

    # MoveIt move_group 节点
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config, 'launch', 'move_group.launch.py')
        )
    )

    # RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config, 'launch', 'moveit_rviz.launch.py')
        )
    )

    # ros2_control 控制器节点（直接写在此文件里）
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml],
        output='screen',
    )

    # 启动 Ti5_node 节点
    ti5_node = Node(
        package='Ti5_moveit_interface',
        executable='Ti5_node',
        name='ti5_moveit_interface',
        output='screen'
    )

    return LaunchDescription([
        rsp_launch,
        controller_manager_node,
        move_group_launch,
        rviz_launch,
        TimerAction(
            period=3.0,
            actions=[ti5_node]
        )
    ])
