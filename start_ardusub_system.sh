#!/bin/bash
# -*- coding: utf-8 -*-
"""
ArduSub ROS系统启动脚本
启动键盘控制节点、状态监控UI和ArduSub接口
"""

echo "=================================="
echo "    ArduSub ROS 控制系统启动"
echo "=================================="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS环境未设置，请先运行 'source /opt/ros/<distro>/setup.bash'"
    exit 1
fi

# 检查Python依赖
echo "检查Python依赖..."
python3 -c "import rospy, tkinter, matplotlib" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "错误: 缺少必要的Python依赖"
    echo "请运行以下命令安装依赖:"
    echo "sudo apt-get install python3-tk python3-matplotlib"
    echo "pip3 install matplotlib"
    exit 1
fi

# 获取当前脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 启动roscore
echo "启动 ROS 核心..."
if ! pgrep -f roscore > /dev/null; then
    roscore &
    ROSCORE_PID=$!
    echo "ROS核心已启动 (PID: $ROSCORE_PID)"
    sleep 3
else
    echo "ROS核心已在运行"
fi

# 设置退出处理函数
cleanup() {
    echo ""
    echo "正在关闭所有进程..."
    
    # 终止所有启动的进程
    if [ ! -z "$ARDUSUB_PID" ]; then
        kill $ARDUSUB_PID 2>/dev/null
    fi
    if [ ! -z "$KEYBOARD_PID" ]; then
        kill $KEYBOARD_PID 2>/dev/null
    fi
    if [ ! -z "$UI_PID" ]; then
        kill $UI_PID 2>/dev/null
    fi
    if [ ! -z "$ROSCORE_PID" ]; then
        kill $ROSCORE_PID 2>/dev/null
    fi
    
    # 清理ROS进程
    pkill -f "ros_ardusub_interface"
    pkill -f "ros_keyboard_control"
    pkill -f "ardusub_status_ui"
    
    echo "所有进程已关闭"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 等待用户选择启动模式
echo ""
echo "请选择启动模式:"
echo "1) 完整模式 (ArduSub接口 + 键盘控制 + 状态UI)"
echo "2) 键盘控制模式 (仅键盘控制 + 状态UI)"
echo "3) 仅状态监控UI"
echo "4) 仅ArduSub接口"
echo "5) 仅键盘控制"
echo ""
read -p "请输入选择 (1-5): " choice

case $choice in
    1)
        echo "启动完整模式..."
        
        # 启动ArduSub接口
        echo "启动 ArduSub ROS 接口..."
        python3 ros_ardusub_interface.py &
        ARDUSUB_PID=$!
        echo "ArduSub接口已启动 (PID: $ARDUSUB_PID)"
        sleep 2
        
        # 启动状态监控UI
        echo "启动状态监控UI..."
        python3 ardusub_status_ui.py &
        UI_PID=$!
        echo "状态监控UI已启动 (PID: $UI_PID)"
        sleep 2
        
        # 启动键盘控制
        echo "启动键盘控制节点..."
        echo "注意: 键盘控制将在单独的终端窗口中运行"
        gnome-terminal -- bash -c "cd '$SCRIPT_DIR' && python3 ros_keyboard_control_node.py; read -p '按回车键关闭...'" &
        KEYBOARD_PID=$!
        
        echo ""
        echo "=================================="
        echo "系统启动完成!"
        echo "- ArduSub接口: PID $ARDUSUB_PID"
        echo "- 状态监控UI: PID $UI_PID" 
        echo "- 键盘控制: 在新终端窗口中运行"
        echo ""
        echo "按 Ctrl+C 关闭所有进程"
        echo "=================================="
        ;;
        
    2)
        echo "启动键盘控制模式..."
        
        # 启动状态监控UI
        echo "启动状态监控UI..."
        python3 ardusub_status_ui.py &
        UI_PID=$!
        echo "状态监控UI已启动 (PID: $UI_PID)"
        sleep 2
        
        # 启动键盘控制
        echo "启动键盘控制节点..."
        python3 ros_keyboard_control_node.py &
        KEYBOARD_PID=$!
        
        echo ""
        echo "键盘控制模式启动完成!"
        echo "按 Ctrl+C 关闭所有进程"
        ;;
        
    3)
        echo "启动状态监控UI..."
        python3 ardusub_status_ui.py &
        UI_PID=$!
        
        echo "状态监控UI已启动"
        echo "按 Ctrl+C 关闭"
        ;;
        
    4)
        echo "启动ArduSub接口..."
        python3 ros_ardusub_interface.py &
        ARDUSUB_PID=$!
        
        echo "ArduSub接口已启动"
        echo "按 Ctrl+C 关闭"
        ;;
        
    5)
        echo "启动键盘控制..."
        python3 ros_keyboard_control_node.py &
        KEYBOARD_PID=$!
        
        echo "键盘控制已启动"
        echo "按 Ctrl+C 关闭"
        ;;
        
    *)
        echo "无效选择，退出"
        exit 1
        ;;
esac

# 等待用户中断
while true; do
    sleep 1
done
