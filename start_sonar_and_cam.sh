#!/bin/bash

# 启动多个ROS节点的脚本
echo "启动声纳处理系统..."

# 在后台启动声纳后处理launch文件
echo "启动声纳后处理..."
roslaunch sonar_image_proc sonar_postproc.launch &
PROC1_PID=$!

# 等待一下让第一个节点启动
sleep 2

# 在后台启动声纳驱动
echo "启动声纳驱动..."
roslaunch oculus_sonar_driver default_ros.launch &
PROC2_PID=$!

# 等待一下让驱动启动
sleep 3

# 在后台启动声纳绘制节点
echo "启动声纳绘制节点..."
rosrun sonar_image_proc draw_sonar_node &
PROC3_PID=$!

# 等待一下让节点启动
sleep 2


# 在后台启动USB摄像头
echo "启动USB摄像头..."
python3 /home/liasorin-clp/workspace/ardusub_control/camera/simple_usb_camera.py &
PROC5_PID=$!

# 等待一下让节点启动
sleep 2

# 在后台启动OAK FFC同步
echo "启动OAK FFC同步..."
roslaunch ros1_oak_ffc_sync oak_ffc_sync.launch &
PROC4_PID=$!

# 等待一下让节点启动
sleep 2

echo "所有节点已启动完成！"
echo "按 Enter 键或输入任何命令来关闭所有进程..."

# 等待用户输入
read -r

# 当用户输入任何内容时，清理所有后台进程
echo "正在关闭所有进程..."
kill $PROC4_PID $PROC5_PID 2>/dev/null
wait
echo "所有进程已关闭。"