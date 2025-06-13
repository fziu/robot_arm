ros2 launch test_description display_robot.launch.py

ros2 run controller_manager ros2_control_node 
加载并激活控制器
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active arm_controller

ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]"



source install/setup.bash
ros2 run moveit_setup_assistant moveit_setup_assistant



ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "
joint_names:
- joint1
points:
- positions: [0.5]
  time_from_start: {sec: 2}
"





<!-- <ros2_control name="test_System" type="system">
        <hardware>
            <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>

        <joint name="joint1">
            <command_interface name="position">
                <param name="min">-6.28</param>
                <param name="max">6.28</param>
            </command_interface>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
    </ros2_control>

    <gazebo>
        <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
            <robot_param>robot_description</robot_param>
            <parameters>$(find test_description)/config/test_controllers.yaml</parameters>
        </plugin>
    </gazebo> -->