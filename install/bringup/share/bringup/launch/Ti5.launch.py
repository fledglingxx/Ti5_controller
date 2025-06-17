import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def load_urdf(pkg_name, urdf_file):
    pkg_path = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_path, 'urdf', urdf_file)
    with open(urdf_path, 'r') as f:
        return f.read()


def load_file(pkg_name, relative_path):
    """Load a file from a package (like SRDF or YAML)"""
    pkg_path = get_package_share_directory(pkg_name)
    file_path = os.path.join(pkg_path, relative_path)
    with open(file_path, 'r') as f:
        return f.read()



def generate_launch_description():

    robot_description = {'robot_description': load_urdf('arms_description', 'T170_ARMS.urdf')}
    robot_description_semantic = {'robot_description_semantic':
                                  load_file('T170_ARMS_moveit_config', 'config/T170_ARMS.srdf')}

    kinematics_yaml = os.path.join(
        get_package_share_directory('T170_ARMS_moveit_config'), 'config', 'kinematics.yaml'
    )

    controllers_yaml = os.path.join(
        get_package_share_directory('T170_ARMS_moveit_config'), 'config','ros2_controllers.yaml'
    )

    joint_limits_yaml = os.path.join(
        get_package_share_directory('T170_ARMS_moveit_config'),'config','joint_limits.yaml'
    )


    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('T170_ARMS_moveit_config'),
                'launch','move_group.launch.py'
            )
        ),
        launch_arguments={
            'robot_description': robot_description['robot_description'],
            'robot_description_semantic': robot_description_semantic['robot_description_semantic'],
            'robot_description_kinematics': kinematics_yaml,
            'robot_description_planning': joint_limits_yaml,
            'controllers_file': controllers_yaml
        }.items()
    )


    arms_display = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('arms_description'),
                    'launch',
                    'display.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
    )


    hardware_node = Node(
        package='Ti5_hardware_interface',
        executable='ti5_hardware_interface',
        name='ti5_hardware_interface',
        parameters=[
            robot_description,
            controllers_yaml
        ],
        output='screen'
    )


    
    control_node = Node(
        package = 'Ti5_arms_controller',
        executable = 'Ti5_arms_controller',
        parameters=[
            robot_description,
            os.path.join(
                get_package_share_directory('bringup'),
                'config',
                'Ti5_controller.yaml'
            )
        ],
        output='screen'
    )



    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster','--controller-manager',   '/controller_manager'],
        output='screen'
    )



    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['Ti5_controller', "-c", '/controller_manager'],
        output='screen'
    )

    ti5_node = Node(
        package='Ti5_moveit_interface',
        executable='Ti5_node',
        name='Ti5_moveit_interface',
        output='screen'
        )
    
    return LaunchDescription([
        arms_display,
        hardware_node,
        move_group_launch,
        control_node,
        joint_broad_spawner,
        robot_controller_spawner,
        ti5_node
    ])
  