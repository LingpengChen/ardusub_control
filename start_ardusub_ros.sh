#!/bin/bash

# ArduSub ROS接口启动脚本
# 使用方法: ./start_ardusub_ros.sh

echo "======================================"
echo "启动 ArduSub ROS 接口"
echo "======================================"

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS环境未设置"
    echo "请先运行: source /opt/ros/noetic/setup.bash"
    echo "或者: source /opt/ros/melodic/setup.bash"
    exit 1
fi

echo "ROS版本: $ROS_DISTRO"

# 检查roscore是否运行
if ! pgrep -x "roscore" > /dev/null; then
    echo "启动 roscore..."
    roscore &
    ROSCORE_PID=$!
    echo "roscore PID: $ROSCORE_PID"
    
    # 等待roscore启动
    sleep 3
else
    echo "roscore 已在运行"
fi

# 设置Python路径
export PYTHONPATH=$PYTHONPATH:$(pwd)

echo "启动ArduSub ROS接口..."
echo "连接字符串: ${ARDUSUB_CONNECTION:=udpin:0.0.0.0:14551}"

# 启动ROS接口
python3 ros_ardusub_interface.py _connection_string:=${ARDUSUB_CONNECTION} &
INTERFACE_PID=$!

echo "ArduSub ROS接口已启动 (PID: $INTERFACE_PID)"
echo ""
echo "可用的ROS话题:"
echo "  发布的话题 (ArduSub状态数据):"
echo "    /ardusub/imu          - IMU数据 (sensor_msgs/Imu)"
echo "    /ardusub/depth        - 深度数据 (std_msgs/Float32)"
echo "    /ardusub/temperature  - 温度数据 (sensor_msgs/Temperature)"
echo "    /ardusub/pressure     - 压力数据 (sensor_msgs/FluidPressure)"
echo "    /ardusub/armed        - 解锁状态 (std_msgs/Bool)"
echo "    /ardusub/odom         - 里程计数据 (nav_msgs/Odometry)"
echo ""
echo "  订阅的话题 (控制指令):"
echo "    /ardusub/cmd_vel      - 速度控制 (geometry_msgs/Twist)"
echo "    /ardusub/thrust_x     - X轴推力 (std_msgs/Float32)"
echo "    /ardusub/thrust_y     - Y轴推力 (std_msgs/Float32)"
echo "    /ardusub/thrust_z     - Z轴推力 (std_msgs/Float32)"
echo "    /ardusub/thrust_yaw   - 偏航推力 (std_msgs/Float32)"
echo "    /ardusub/target_depth - 目标深度 (std_msgs/Float32)"
echo "    /ardusub/target_attitude - 目标姿态 (geometry_msgs/Vector3)"
echo "    /ardusub/servo_control - 舵机控制 (geometry_msgs/Vector3)"
echo "    /ardusub/gimbal       - 云台控制 (geometry_msgs/Vector3)"
echo "    /ardusub/lights       - 灯光控制 (std_msgs/Int32)"
echo "    /ardusub/arm          - 解锁控制 (std_msgs/Bool)"
echo "    /ardusub/set_mode     - 模式切换 (std_msgs/String)"
echo ""
echo "按 Ctrl+C 退出"

# 等待中断信号
trap 'echo "正在关闭..."; kill $INTERFACE_PID 2>/dev/null; if [ ! -z "$ROSCORE_PID" ]; then kill $ROSCORE_PID 2>/dev/null; fi; exit' INT

# 保持脚本运行
wait $INTERFACE_PID
