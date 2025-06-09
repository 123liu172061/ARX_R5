import socket

# 接收端口设置
UDP_IP = "10.192.1.10"   # 监听本地所有网卡
UDP_PORT = 12345     # Unity 中配置的端口

# 创建 Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"🔌 Listening on {UDP_IP}:{UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)  # 每次最多接收 1024 字节
    try:
        decoded = data.decode('utf-8')
        parts = decoded.strip().split(",")

        if len(parts) != 9:
            print(f"Invalid data: {decoded}")
            continue

        x, y, z = map(float, parts[0:3])
        qw, qx, qy, qz = map(float, parts[3:7])
        trigger = int(parts[7])
        grip = int(parts[8])

        print(f"""
        Received from {addr}:
        Position : ({x:.3f}, {y:.3f}, {z:.3f})
        Rotation : (w={qw:.3f}, x={qx:.3f}, y={qy:.3f}, z={qz:.3f})
        Trigger  : {trigger}
        Grip     : {grip}
                """)

    except Exception as e:
        print(f" Error: {e}")

