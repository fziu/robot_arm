import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    # 获取urdf、config文件
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="cat")]), " ",
        PathJoinSubstitution([FindPackageShare("test_description"), "urdf", "test.urdf"])]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("test_description"), "config", "test_controllers.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("test_description"), "config", "display_robot_model.rviz"]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"])
    )

    spawn_entity = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments = ["-topic", "/robot_description", "-entity", "robot"],
        output="screen"
    )

    # 执行完target_action，再执行on_exit
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_robot_controller_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[robot_controller_spawner],
        )
    )

    pkg_share = os.pathsep + os.path.join(get_package_prefix("test_description"), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:  # 如果你修改了~/.bashrc, 就会执行这个
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:  # 注意此处gazebo-11修改为你的gazebo版本
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share
    
    return LaunchDescription([
        gazebo,
        robot_state_pub_node,
        spawn_entity,
        delay_robot_controller_spawner_after_spawn_entity,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner
    ])