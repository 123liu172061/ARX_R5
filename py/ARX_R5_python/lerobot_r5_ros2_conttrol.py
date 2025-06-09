#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from bimanual import SingleArm
import numpy as np

class ShoubingArmController(Node):
    def __init__(self):
        super().__init__('shoubing_arm_controller')

        # Initialize the arm
        arm_config = {
            "can_port": "can1",
            "type": 0,
        }
        self.single_arm = SingleArm(arm_config)
        self.prev_position = None
        self.allow_control = False

        # Subscribe to joint_position control topic
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/master/joint_right',
            self.joint_position_callback,
            10
        )

        # Create publisher for joint states
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/puppet/joint_right',
            10
        )

        # Create a timer for publishing joint states at 50Hz
        timer_period = 1.0 / 20.0  # seconds
        self.pub_timer = self.create_timer(timer_period, self.publish_joint_states) # 20Hz

        self.control_timer = self.create_timer(0.02, self.control_timer_callback)  # 50Hz

        # 初始化属性
        self.start_joints = None
        self.target_joints = None
        self.start_catch = None
        self.target_catch = None
        self.step = 0
        self.total_steps = 5
        self.allow_control = False


        self.get_logger().info("Shoubing control node started, waiting for commands...")

    # 线性插值
    def lerp(self, start, end, alpha):
            return start + alpha * (end - start)
    # 获取目标关节位置
    def joint_position_callback(self, msg):
        if len(msg.position) >= 7:
            self.start_joints = np.array(self.single_arm.get_joint_positions())
            self.target_joints = np.array(msg.position[:6])
            print("target_joints",self.target_joints)
            self.start_catch = self.start_joints[6]
            self.target_catch = msg.position[6]
            self.step = 0
            self.total_steps = 5
            self.allow_control = True

    # 定时器里进行插值控制
    def control_timer_callback(self):
        if self.allow_control and self.step < self.total_steps:
            alpha = (self.step + 1) / self.total_steps
            interp_joints = self.lerp(self.start_joints[:6], self.target_joints, alpha)
            interp_catch = self.lerp(self.start_catch, self.target_catch, alpha)
            self.single_arm.set_joint_positions(interp_joints.tolist())
            self.single_arm.set_catch_pos(interp_catch)
            self.step += 1
        elif self.step >= self.total_steps:
            self.allow_control = False


    def publish_joint_states(self):
        # Get current joint positions
        joint_positions = self.single_arm.get_joint_positions()
        #print("joint_positions:", joint_positions)
        # Create and fill JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"joint_{i}" for i in range(7)]
        joint_state_msg.position = joint_positions[:7]

        # Publish the message
        self.joint_state_pub.publish(joint_state_msg)

        # Optional: log at ~1Hz
        now_sec = self.get_clock().now().seconds_nanoseconds()[0]
        # if now_sec % 1 == 0:
        #     self.get_logger().info(f"Published joint states: {joint_positions[:7]}")

def main(args=None):
    rclpy.init(args=args)
    controller = ShoubingArmController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
