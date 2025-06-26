#!/usr/bin/env python3
import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    # 参数设置
    ld.add_action(
        DeclareLaunchArgument('use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true')
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(
        DeclareLaunchArgument('use_rviz', default_value='true',
            description='Use rviz if true')
    )
    use_rviz = LaunchConfiguration('use_rviz')

    # 加载配置文件
    moveit_config_dir = os.path.join(
        get_package_share_directory('test_moveit_config'), 'config')
    
    ## Rviz config save file
    rviz_config = os.path.join(moveit_config_dir, "moveit.rviz")

    ## Ros2 controllers
    ros2_controllers = os.path.join(moveit_config_dir, "ros2_controllers.yaml")

    ## Robot description
    robot_description_config = xacro.process_file(
        os.path.join(moveit_config_dir, "test_robot.urdf.xacro")
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    # move_group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("test_bringup"), "/launch", "/move_group.launch.py"])        
    )

    # robot_state_publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ros2_control_node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers, {'use_sim_time': use_sim_time}],
        remappings=[
            ('~/cmd_vel_unstamped', 'cmd_vel'),
            ('~/odom', 'odom')
        ],
        output="both",
    )

    # joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # arm_controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"])
    )

    # spawn_entity  
    spawn_entity = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments=['-topic', '/robot_description', '-entity', 'robot'],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    delay_acs_after_jsbs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    delay_move_after_jsbs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[move_group],
        )
    )
    delay_rviz_after_jsbs = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    ld.add_action(robot_state_pub_node)
    ld.add_action(gazebo)
    ld.add_action(spawn_entity)

    ld.add_action(control_node)
    ld.add_action(joint_state_broadcaster_spawner)

    ld.add_action(delay_acs_after_jsbs)
    ld.add_action(delay_move_after_jsbs)
    ld.add_action(delay_rviz_after_jsbs)


    pkg_share = os.pathsep + os.path.join(get_package_prefix("test_description"), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:  # 如果你修改了~/.bashrc, 就会执行这个
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:  # 注意此处gazebo-11修改为你的gazebo版本
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share

    return ld