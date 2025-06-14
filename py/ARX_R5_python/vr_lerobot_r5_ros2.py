import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from bimanual import SingleArm
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading

def clamp_xyz(pos_xyz, min_vals, max_vals):
    return np.clip(pos_xyz, min_vals, max_vals)

class ShoubingArmController(Node):
    def __init__(self):
        super().__init__('shoubing_arm_controller')

        arm_config = {
            "can_port": "can1",
            "type": 0,
        }
        self.single_arm = SingleArm(arm_config)
        self.prev_position = None
        self.xyzrpy = np.zeros(6)
        self.allow_control = False

        self.pose_sub = self.create_subscription(
            Pose,
            '/shoubing/pose',
            self.pose_callback,
            10
        )

        self.state_sub = self.create_subscription(
            Int32MultiArray,
            '/shoubing/state',
            self.state_callback,
            10
        )
        self.home_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.joint_target_state_pub = self.create_publisher(JointState, 'joint_states_target', 10)
        self.joint_now_state_pub = self.create_publisher(JointState, 'joint_states_now', 10)
        self.joint_names = [f'joint{i}' for i in range(7)]
        
        # 启动键盘监听线程
        threading.Thread(target=self.keyboard_listener, daemon=True).start()
        self.get_logger().info("Shoubing 控制节点已启动，等待手柄控制...")

    def pose_callback(self, msg):
        current_pose = self.single_arm.get_ee_pose_xyzrpy()
        curr_pos = current_pose[:3]
        curr_rpy = current_pose[3:]

        new_pos = np.array([msg.position.z, msg.position.x, msg.position.y])
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        r = R.from_quat(quaternion)
        new_rpy = r.as_euler('zyx', degrees=False)

        if self.prev_position is None:
            self.prev_position = new_pos
            return

        delta_pos = new_pos - self.prev_position
        self.prev_position = new_pos

        # vr遥操控制参数
        pos_gain = 4.5
        new_xyzrpy = np.zeros(6)
        new_pos_xyz = curr_pos + delta_pos * pos_gain

        min_vals = np.array([0.0, -0.4, -0.1])
        max_vals = np.array([0.5, 0.4, 0.3])
        new_pos_xyz = clamp_xyz(new_pos_xyz, min_vals, max_vals)

        new_xyzrpy[:3] = new_pos_xyz
        new_xyzrpy[3] = new_rpy[0] * 0.5
        new_xyzrpy[4] = new_rpy[2] * 0.7  #由于坐标系未对齐，所以rpy顺序 机械臂和手柄是有错位
        new_xyzrpy[5] = new_rpy[1] * 0.8
    
        pos_now =  self.single_arm.get_joint_positions()
        print("pos_now:", pos_now)
        # 1.发布now_states
        joint_now_curr = self.single_arm.get_joint_currents()  #lerobot中不需要此电流数据，无用
        joint_now_state_msg = JointState()
        joint_now_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_now_state_msg.name = self.joint_names
        joint_now_state_msg.position = pos_now      # 仅该数据有用  相当于从臂关节位置数据
        joint_now_state_msg.effort = joint_now_curr
        self.joint_now_state_pub.publish(joint_now_state_msg)

        self.get_logger().info(f"控制机械臂位置增量: {delta_pos}, 新位姿: {new_xyzrpy[:3]}")
        # pos_solve = inverse_kinematics(new_xyzrpy)  # 6个关节位置，还差手抓关节 没法用，会经常逆解不了，不知道为何，按理末端控制也用了内部逆解
        if self.allow_control:
            self.single_arm.set_ee_pose_xyzrpy(new_xyzrpy) # 控制接口
    

        # 2.发布now_states target_states
        pos_target =  self.single_arm.get_joint_positions()
        print("pos_target:", pos_target)
        # 1.发布target_states
        joint_target_curr = self.single_arm.get_joint_currents()  #lerobot中不需要此数据，无用
        joint_target_state_msg = JointState()
        joint_target_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_target_state_msg.name = self.joint_names
        joint_target_state_msg.position = pos_target
        joint_target_state_msg.effort = joint_target_curr
        self.joint_target_state_pub.publish(joint_target_state_msg)

    def state_callback(self, msg):
        if len(msg.data) >= 2:
            trigger, grip = msg.data[0], msg.data[1]
            self.allow_control = (grip == 1) # 使能功能，当扳机键按下则使能机械臂受控
            self.get_logger().info(f"手柄状态 -> Trigger: {trigger}, Grip: {grip}, 控制允许: {self.allow_control}")
            joint_position = self.single_arm.get_joint_positions()
            joint_curr = self.single_arm.get_joint_currents()


            if trigger == 1:
                # 保护夹爪，但是其实sdk里已经有保护的
                if joint_curr[6] < -1.0:
                    self.single_arm.set_catch_pos(joint_position[6])
                else:
                    self.single_arm.set_catch_pos(0.0)
                self.get_logger().info("Trigger 按下，夹爪闭合")
            else:
                self.single_arm.set_catch_pos(5.0)
                self.get_logger().info("Trigger 松开，夹爪张开")

    def keyboard_listener(self):
        while rclpy.ok():
            try:
                _ = input()  # 按下回车后执行
                self.get_logger().info("收到回车，机械臂缓慢回到初始点...")

                current_pose = np.array(self.single_arm.get_ee_pose_xyzrpy())
                target_pose = np.array(self.home_pose)
                steps = 50
                rate = self.create_rate(50)

                for i in range(1, steps + 1):
                    alpha = i / steps
                    intermediate_pose = (1 - alpha) * current_pose + alpha * target_pose
                    self.single_arm.set_ee_pose_xyzrpy(intermediate_pose.tolist())
                    rate.sleep()

                self.get_logger().info("机械臂已回到初始点。")
            except Exception as e:
                self.get_logger().warn(f"键盘监听错误: {e}")

def main():
    rclpy.init()
    controller = ShoubingArmController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
