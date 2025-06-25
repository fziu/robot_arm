# 机械臂控制
[鱼香ros](https://zhuanlan.zhihu.com/p/567246503)

[Stv.Li](https://zhuanlan.zhihu.com/p/616711291)
## test_ws
ros2 launch test_description display_robot.launch.py

ros2 control list_controllers
### forward_position_controller
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0]"
### joint_trajectory_position_controller
```
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
```
### test_control
ros2 run test_control test_control --ros-args -p use_sim_time:=true

## moveit_ws
source install/setup.bash

ros2 run moveit_setup_assistant moveit_setup_assistant
```
action_ns: follow_joint_trajectory
default: true
type: FollowJointTrajectory
```

ros2 launch test_moveit_config demo_gazebo.launch.py

ros2 param set /move_group use_sim_time true