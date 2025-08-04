#!/bin/bash

# ArduSub ROS接口安装脚本

echo "======================================"
echo "ArduSub ROS接口 - 依赖安装脚本"
echo "======================================"

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then
    echo "请不要使用sudo运行此脚本"
    exit 1
fi

# 检查操作系统
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS=$NAME
    VER=$VERSION_ID
else
    echo "无法确定操作系统版本"
    exit 1
fi

echo "检测到操作系统: $OS $VER"

# 检查ROS是否已安装
if [ -z "$ROS_DISTRO" ]; then
    echo "警告: ROS环境变量未设置"
    
    # 尝试检测已安装的ROS版本
    if [ -d "/opt/ros/noetic" ]; then
        echo "检测到ROS Noetic"
        export ROS_DISTRO=noetic
        source /opt/ros/noetic/setup.bash
    elif [ -d "/opt/ros/melodic" ]; then
        echo "检测到ROS Melodic"
        export ROS_DISTRO=melodic
        source /opt/ros/melodic/setup.bash
    else
        echo "未检测到ROS安装"
        echo "是否要安装ROS Noetic? (y/n)"
        read -r response
        if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
            echo "安装ROS Noetic..."
            
            # 添加ROS软件源
            sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
            
            # 添加密钥
            curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
            
            # 更新包列表
            sudo apt update
            
            # 安装ROS
            sudo apt install -y ros-noetic-desktop-full
            
            # 初始化rosdep
            sudo rosdep init
            rosdep update
            
            # 设置环境
            echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
            source /opt/ros/noetic/setup.bash
            
            export ROS_DISTRO=noetic
            
            echo "ROS Noetic安装完成"
        else
            echo "请手动安装ROS后再运行此脚本"
            exit 1
        fi
    fi
else
    echo "ROS版本: $ROS_DISTRO"
fi

# 检查Python3
if ! command -v python3 &> /dev/null; then
    echo "Python3未安装，正在安装..."
    sudo apt update
    sudo apt install -y python3 python3-pip
fi

echo "Python3版本: $(python3 --version)"

# 检查pip3
if ! command -v pip3 &> /dev/null; then
    echo "pip3未安装，正在安装..."
    sudo apt install -y python3-pip
fi

# 安装Python依赖
echo "安装Python依赖..."

# 安装pymavlink
pip3 install --user pymavlink

# 安装其他可能需要的包
pip3 install --user numpy

# 验证安装
echo "验证安装..."

python3 -c "import pymavlink; print('pymavlink版本:', pymavlink.__version__)" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ pymavlink安装成功"
else
    echo "✗ pymavlink安装失败"
    exit 1
fi

# 检查ROS Python包
python3 -c "import rospy; print('rospy可用')" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ rospy可用"
else
    echo "✗ rospy不可用，可能需要安装python3-rospy"
    if [[ "$ROS_DISTRO" == "noetic" ]]; then
        sudo apt install -y python3-rospy python3-roslib python3-rospkg
    else
        sudo apt install -y python-rospy python-roslib python-rospkg
    fi
fi

# 创建ROS环境设置脚本
cat > setup_ros_env.sh << 'EOF'
#!/bin/bash
# ROS环境设置脚本

# 检测并设置ROS环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo "ROS Noetic环境已加载"
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    source /opt/ros/melodic/setup.bash
    echo "ROS Melodic环境已加载"
else
    echo "未找到ROS安装"
    exit 1
fi

# 设置Python路径
export PYTHONPATH=$PYTHONPATH:$(pwd)

echo "ROS环境设置完成"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
EOF

chmod +x setup_ros_env.sh

echo ""
echo "======================================"
echo "安装完成！"
echo "======================================"
echo ""
echo "使用方法："
echo "1. 设置ROS环境:"
echo "   source setup_ros_env.sh"
echo ""
echo "2. 启动ArduSub ROS接口:"
echo "   ./start_ardusub_ros.sh"
echo ""
echo "3. 运行测试:"
echo "   python3 test_ardusub_ros.py"
echo ""
echo "注意："
echo "- 确保ArduSub在端口14551上运行"
echo "- 如果首次运行，请重新启动终端或运行 'source ~/.bashrc'"
echo "- 详细使用说明请查看 README.md"
echo ""
