import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import time
import numpy as np
from copy import copy

class test_control(Node):
    def __init__(self):
        super().__init__('test_control')

        self.subscription = self.create_subscription(
            JointTrajectoryControllerState, '/joint_trajectory_position_controller/controller_state', self.controller_callback, 10)
        self.publisher = self.create_publisher(
            JointTrajectory, '/joint_trajectory_position_controller/joint_trajectory', 10)
        
        self.init_time = None
        self.init_position = None
    
    def controller_callback(self, msg):
        if len(msg.feedback.positions) <= 0:
            return

        if self.init_time is None:
            self.init_time = time.time()
            self.init_position = copy(msg.feedback.positions)

        correction = 3.14 * (np.sin(0.5 * (time.time() - self.init_time)))
        tar_pos0 = self.init_position[0] + correction
        # tar_pos1 = self.init_position[1] + correction
        # tar_pos2 = self.init_position[2] + correction
        # tar_pos3 = self.init_position[3] + correction
        # tar_pos4 = self.init_position[4] + correction
        # tar_pos5 = self.init_position[5] + correction
        tar_pos1 = -1.57
        tar_pos2 = 1.57
        tar_pos3 = 0.0
        tar_pos4 = 1.57
        tar_pos5 = 0.0


        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        point = JointTrajectoryPoint()
        point.positions = [tar_pos0, tar_pos1, tar_pos2, tar_pos3, tar_pos4, tar_pos5]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        msg.points.append(point)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = test_control()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()