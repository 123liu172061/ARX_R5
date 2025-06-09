import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
from bimanual import SingleArm
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState



def clamp_xyz(pos_xyz, min_vals, max_vals):
    return np.clip(pos_xyz, min_vals, max_vals)


class ShoubingArmController(Node):
    def __init__(self):
        super().__init__('shoubing_arm_controller')

        # 初始化机械臂
        arm_config = {
            "can_port": "can1",
            "type": 0,
        }
        self.single_arm = SingleArm(arm_config)
        self.prev_position = None
        self.xyzrpy = np.zeros(6)
        self.allow_control = False  # 用于判断是否允许控制机械臂

        # 订阅手柄位姿
        self.subscription = self.create_subscription(
            Pose,
            '/shoubing/pose',
            self.pose_callback,
            10
        )

        # 订阅 trigger 和 grip 状态
        self.state_subscription = self.create_subscription(
            Int32MultiArray,
            '/shoubing/state',
            self.state_callback,
            10
        )
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [f'joint{i}' for i in range(7)]  # 根据实际电机数量修改


        self.get_logger().info("Shoubing 控制节点已启动，等待手柄控制...")

    def pose_callback(self, msg: Pose):


        # 获取当前机械臂位姿
        current_pose = self.single_arm.get_ee_pose_xyzrpy()
        
        # 获取当前位姿的 xyz 和 rpy
        curr_pos = current_pose[:3]
        curr_rpy = current_pose[3:]
        #print("当前位置：",curr_pos)
        # 当前收到的位姿数据
        new_pos = np.array([msg.position.z, msg.position.x, msg.position.y])
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        #print("姿态：",quaternion)
        r = R.from_quat(quaternion)
        new_rpy = r.as_euler('zyx', degrees=False)  # 得到 roll, pitch, yaw
        #print("new_rpy:", new_rpy)

        if self.prev_position is None :
            self.prev_position = new_pos
            return

        # 增量（当前位置与前一位置的差值）
        delta_pos = new_pos - self.prev_position
        self.prev_position = new_pos  # 更新为当前帧

        # 组合增量控制（可调倍率）
        pos_gain = 4.5
        # 增量应用到位置（当前的位置 + 增量）
        new_xyzrpy = np.zeros(6)  # 确保有六个元素
        new_pos_xyz = curr_pos + delta_pos * pos_gain #相对量
        # 对位置进行限幅
        min_vals = np.array([0.0, -0.4, -0.1])  # 最小值（单位：米）
        max_vals = np.array([0.5, 0.4, 0.3])    # 最大值（单位：米）
        new_pos_xyz = clamp_xyz(new_pos_xyz, min_vals, max_vals)
        # 位置赋值
        new_xyzrpy[:3] = new_pos_xyz

        # 姿态赋值  因为坐标系不同，所以不是按顺序赋值，通过系数来控制灵敏度
        new_xyzrpy[3] = new_rpy[0]*0.5  #0.5
        new_xyzrpy[4] = new_rpy[2]*0.7  #0.7
        new_xyzrpy[5] = (new_rpy[1])*0.8  #有偏移，更新手柄系统后，很奇怪
        #new_xyzrpy[3:] = new_rpy #绝对量
        self.get_logger().info(f"控制机械臂位置增量: {delta_pos}, 新位姿: {new_xyzrpy[:3]}")
        if self.allow_control:
            # 发送新的目标位姿到机械臂
            # print("kongzhi ")
            self.single_arm.set_ee_pose_xyzrpy(new_xyzrpy)
    

    def state_callback(self, msg: Int32MultiArray):
        if len(msg.data) >= 2:
            trigger, grip = msg.data[0], msg.data[1]
            self.allow_control = (grip == 1)
            self.get_logger().info(f"手柄状态 -> Trigger: {trigger}, Grip: {grip}, 控制允许: {self.allow_control}")
            joint_position =self.single_arm.get_joint_positions()
            joint_curr = self.single_arm.get_joint_currents()
            print("joint_curr:", joint_curr)
            # 发布 joint_states
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = joint_position
            joint_state_msg.effort = joint_curr
            # 可以添加 joint_state_msg.velocity = [...] 如果你有的话
            self.joint_state_pub.publish(joint_state_msg)

            # 控制夹爪：trigger 按下为1时闭合，松开为0时打开
            if trigger == 1:
                if joint_curr[6]<-1.0:
                        self.single_arm.set_catch_pos(joint_position[6])  # 如果力矩过大，则控制当前位置
                else:
                    self.single_arm.set_catch_pos(0.0)  # 0.0 表示夹爪闭合
                self.get_logger().info("Trigger 按下，夹爪闭合")
            else:
                self.single_arm.set_catch_pos(5.0)  # 1.0 表示夹爪张开
                self.get_logger().info("Trigger 松开，夹爪张开")

def main(args=None):
    rclpy.init(args=args)
    node = ShoubingArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

