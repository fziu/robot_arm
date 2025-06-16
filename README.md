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

ros2 run test_control test_control --ros-args -p use_sim_time:=true



source install/setup.bash
ros2 run moveit_setup_assistant moveit_setup_assistant






header:
  stamp:
    sec: 1750037754
    nanosec: 919165232
  frame_id: ''
joint_names:
- joint1
points:
- positions:
  - 1.57
  velocities:
  - 0.0
  accelerations: []
  effort: []
  time_from_start:
    sec: 2
    nanosec: 0
---
header:
  stamp:
    sec: 1750037754
    nanosec: 929570936
  frame_id: ''
joint_names:
- joint1
points:
- positions:
  - 1.57
  velocities:
  - 0.0
  accelerations: []
  effort: []
  time_from_start:
    sec: 2
    nanosec: 0


header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
- joint1
points:
- positions:
  - 0.0
  velocities:
  - 0.0
  accelerations: []
  effort: []
  time_from_start:
    sec: 2
    nanosec: 0
---
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
- joint1
points:
- positions:
  - 0.0
  velocities:
  - 0.0
  accelerations: []
  effort: []
  time_from_start:
    sec: 2
    nanosec: 0