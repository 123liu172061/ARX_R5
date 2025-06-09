import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray

import socket

class ShoubingReceiver(Node):
    def __init__(self):
        super().__init__('shoubing_receiver')

        # 发布 Pose（不带 header）
        self.pose_pub = self.create_publisher(Pose, '/shoubing/pose', 10)
        # 发布 trigger 与 grip 状态
        self.state_pub = self.create_publisher(Int32MultiArray, '/shoubing/state', 10)

        # 初始化 UDP Socket
        self.udp_ip = "192.168.3.39"
        self.udp_port = 12345
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.get_logger().info(f"Listening on {self.udp_ip}:{self.udp_port}...")

        # 定时器，循环读取 socket 数据
        self.timer = self.create_timer(0.01, self.receive_data)  # 100Hz

    def receive_data(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            decoded = data.decode('utf-8')
            parts = decoded.strip().split(',')

            if len(parts) != 9:
                self.get_logger().warn(f"Invalid data: {decoded}")
                return

            x, y, z = map(float, parts[0:3])
            qw, qx, qy, qz = map(float, parts[3:7])
            trigger = int(parts[7])
            grip = int(parts[8])

            # 发布 Pose（无 header）
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            pose_msg.orientation.w = qw
            pose_msg.orientation.x = qx
            pose_msg.orientation.y = qy
            pose_msg.orientation.z = qz
            self.pose_pub.publish(pose_msg)

            # 发布 trigger 和 grip 状态
            state_msg = Int32MultiArray()
            state_msg.data = [trigger, grip]
            self.state_pub.publish(state_msg)

        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ShoubingReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()