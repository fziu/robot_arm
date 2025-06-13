ros2 launch test_description display_robot.launch.py

ros2 control list_controllers
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0]"

ros2 topic pub /joint_trajectory_position_controller/joint_trajectory trajectory_msgs/JointTrajectory '{
  "joint_names": ["joint1"],
  "points": [
    {
      "positions": [0.0],
      "velocities": [0.0],
      "time_from_start": {"sec": 2}
    }
  ]
}'


source install/setup.bash
ros2 run moveit_setup_assistant moveit_setup_assistant