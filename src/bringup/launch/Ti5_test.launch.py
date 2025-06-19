from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moveit_config_pkg = FindPackageShare('T170_ARMS_moveit_config')
    ti5_interface_pkg = FindPackageShare('Ti5_moveit_interface')

    controllers_yaml = PathJoinSubstitution([
        moveit_config_pkg, 'config', 'ros2_controllers.yaml'
    ])

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            moveit_config_pkg, 'config', 'T170_ARMS.urdf.xacro'
        ]),
        ' initial_positions_file:=',
        PathJoinSubstitution([
            moveit_config_pkg, 'config', 'initial_positions.yaml'
        ])
    ])
    robot_description = {'robot_description': robot_description_content}

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'rsp.launch.py'])
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'move_group.launch.py'])
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'moveit_rviz.launch.py'])
        )
    )

    # 启动 ros2_control_node（硬件控制系统）
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='both'
    )

    # 控制器 spawner 节点（通过事件触发）
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['R_group_controller'],
        output='screen'
    )

    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['L_group_controller'],
        output='screen'
    )

    # 在 controller_manager 启动后再启动所有 spawner
    controller_spawner_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        right_arm_controller_spawner,
                        left_arm_controller_spawner,
                    ]
                )
            ]
        )
    )

    # 自定义 moveit 接口节点（建议延迟启动）
    ti5_node = Node(
        package='Ti5_moveit_interface',
        executable='Ti5_node',
        name='ti5_moveit_interface',
        output='screen'
    )

    return LaunchDescription([
        rsp_launch,
        controller_manager_node,
        controller_spawner_handler,  # 保证 controller_manager 启动后再加载控制器
        move_group_launch,
        rviz_launch,
        # TimerAction(period=3.0, actions=[ti5_node])  # 接口节点延迟启动
    ])
