import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros
import launch_ros.parameter_descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_prefix
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 获取路径
    package_path = get_package_share_directory("test_description")
    urdf_path = os.path.join(package_path, "urdf", "test.urdf")
    rviz_config_path = os.path.join(package_path, "config", "display_robot_model.rviz")
    controllers_path = os.path.join(package_path, "config", "test_controllers.yaml")

    # 声明urdf目录参数
    declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name = "model", default_value = str(urdf_path), description = "加载的模型文件路径"
    )

    # 通过文件路径，获取内容，并转换成参数值对象，传入robot_state_publisher
    substitutions_command_result = launch.substitutions.Command(["cat ",
        launch.substitutions.LaunchConfiguration("model")])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result, value_type = str)
    
    robot_description = {"robot_description": robot_description_value}

    robot_state_publisher = launch_ros.actions.Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [robot_description]
    )

    joint_state_publisher = launch_ros.actions.Node(
        package = "joint_state_publisher",
        executable = "joint_state_publisher"
    )

    gazebo = launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_package_share_directory("gazebo_ros"), "/launch", "/gazebo.launch.py"])
    )

    spawn_entity = launch_ros.actions.Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments = ["-topic", "/robot_description", "-entity", "robot"]
    )

    rviz2_node = launch_ros.actions.Node(
        package = "rviz2",
        executable = "rviz2",
        arguments = ["-d", rviz_config_path]
    )

    control_manager_node = launch_ros.actions.Node(
        package = "controller_manager",
        executable = "ros2_control_node",
        parameters = [controllers_path],
        remappings=[("~/robot_description", "/robot_description")],
        output = 'screen'
    )

    control_joint_node = launch_ros.actions.Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output = 'screen'
    )

    # control_forward_node = launch_ros.actions.Node(
    #     package = "controller_manager",
    #     executable = "spawner",
    #     arguments = ["forward_position_controller", "--controller-manager", "/controller_manager"],
    #     output = 'screen'
    # )

    control_trajectory_node = launch_ros.actions.Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = ["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output = 'screen'
    )

    # load_joint_state_broadcaster = launch.actions.ExecuteProcess(
    #     cmd = ["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
    #     output = "screen"
    # )

    # load_forward_position_controller = launch.actions.ExecuteProcess(
    #     cmd = ["ros2", "control", "load_controller", "--set-state", "active", "forward_position_controller"],
    #     output = "screen"
    # )

    pkg_share = os.pathsep + os.path.join(get_package_prefix("test_description"), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:  # 如果你修改了~/.bashrc, 就会执行这个
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:  # 注意此处gazebo-11修改为你的gazebo版本
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share

    return launch.LaunchDescription([
        declare_arg_model_path,

        # launch.actions.RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[load_joint_state_broadcaster],
        #     )
        # ),
        # launch.actions.RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_broadcaster,
        #         on_exit=[load_forward_position_controller],
        #     )
        # ),

        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,

        control_manager_node,
        control_joint_node,
        # control_forward_node,
        control_trajectory_node,

        rviz2_node
    ])