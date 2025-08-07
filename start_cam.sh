#!/bin/bash

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