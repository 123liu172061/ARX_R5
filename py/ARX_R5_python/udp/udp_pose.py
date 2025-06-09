import socket

# 服务器IP和端口（与你Unity中设置一致）
UDP_IP = "10.192.1.10"  # 监听所有网卡
UDP_PORT = 12345

# 创建Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on UDP {UDP_IP}:{UDP_PORT}...")

try:
    while True:
        data, addr = sock.recvfrom(1024)  # 接收最多1024字节
        decoded_data = data.decode("utf-8")

        try:
            # 解析位置和旋转数据
            values = list(map(float, decoded_data.split(",")))
            if len(values) == 7:
                pos = values[0:3]
                rot = values[3:7]
                print(f"From {addr}:")
                print(f"  Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
                print(f"  Rotation: w={rot[0]:.3f}, x={rot[1]:.3f}, y={rot[2]:.3f}, z={rot[3]:.3f}")
            else:
                print(f"[Invalid Format] {decoded_data}")
        except ValueError:
            print(f"[Decode Error] Raw: {decoded_data}")

except KeyboardInterrupt:
    print("Receiver stopped.")

finally:
    sock.close()
