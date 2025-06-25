import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    ld = LaunchDescription()

    moveit_config = MoveItConfigsBuilder("test_robot", package_name="test_moveit_config").to_moveit_configs()
    launch_package_path = moveit_config.package_path

    ld.add_action(
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            )
        )
    )

    # Robot description
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("test_moveit_config"),
            "config",
            "test_robot.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    controller_manager_config = PathJoinSubstitution(
        [
            FindPackageShare('test_moveit_config'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('test_moveit_config'),
            'config',
            'moveit.rviz'
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_manager_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('~/cmd_vel_unstamped', 'cmd_vel'),
            ('~/odom', 'odom')
        ],
        output="both",
    )
    ld.add_action(control_node)

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(robot_state_pub_node)

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    ld.add_action(joint_state_broadcaster_spawner)

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    delay_arm_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    ld.add_action(delay_arm_controller_spawner_after_joint_state_broadcaster_spawner)

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    ld.add_action(delay_rviz_after_joint_state_broadcaster_spawner)


    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare('gazebo_ros'),
                            'launch',
                            'gazebo.launch.py'
                        ]
                    )
                ]
            )
        )
    )

    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}
    
    ld.add_action(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', '/robot_description',
                '-entity', 'robot',
                '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
                '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y'],
                ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
    )

    pkg_share = os.pathsep + os.path.join(get_package_prefix("test_description"), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:  # 如果你修改了~/.bashrc, 就会执行这个
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:  # 注意此处gazebo-11修改为你的gazebo版本
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share

    return ld