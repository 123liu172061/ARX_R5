## 用于lft电脑，ros2

# 开起can
cd /home/lft/workspace/ros_project/R5_vr_ros2/ARX_CAN
conda activate r5
bash test_protect.sh

# 启动udp接收vr数据节点
conda activate r5
cd /home/lft/workspace/ros_project/R5_vr_ros2/py/ARX_R5_python
bash udp_lft.sh

# 启动机械臂控制节点 纯遥操
cd /home/lft/workspace/ros_project/R5_vr_ros2/py/ARX_R5_python
source ./setup.sh
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
conda activate r5
python getvr.py


# lerobot框架下遥操收集数据驱动端  (带回车回到home点)
cd /home/lft/workspace/ros_project/R5_vr_ros2/py/ARX_R5_python
source ./setup.sh
conda activate r5
bash runvr_lft.sh


# lerobot 推理 (优化了，加入线性插值，稍微没那么抖，但是会一顿一顿的 )
cd /home/lft/workspace/ros_project/R5_vr_ros2/py/ARX_R5_python
source ./setup.sh
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
conda activate r5
python lerobot_r5_ros2_conttrol.py



