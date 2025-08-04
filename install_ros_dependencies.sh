#!/bin/bash
# ArduSub ROS控制系统依赖安装脚本

echo "=================================="
echo "  ArduSub ROS控制系统依赖安装"
echo "=================================="

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then
    echo "请不要以root用户运行此脚本"
    exit 1
fi

# 检查系统版本
echo "检查系统版本..."
if command -v lsb_release &> /dev/null; then
    DISTRO=$(lsb_release -si)
    VERSION=$(lsb_release -sr)
    echo "检测到系统: $DISTRO $VERSION"
else
    echo "无法检测系统版本，假设为Ubuntu"
    DISTRO="Ubuntu"
fi

# 更新包管理器
echo "更新包管理器..."
sudo apt-get update

# 安装基础依赖
echo "安装基础依赖..."
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-tk \
    git \
    curl \
    wget \
    build-essential \
    cmake

# 检查ROS是否已安装
if [ -z "$ROS_DISTRO" ]; then
    echo "未检测到ROS环境，开始安装ROS..."
    
    # 根据系统版本选择ROS版本
    if [[ "$VERSION" == "20.04" ]]; then
        ROS_VERSION="noetic"
    elif [[ "$VERSION" == "18.04" ]]; then
        ROS_VERSION="melodic"
    else
        echo "不支持的Ubuntu版本，请手动安装ROS"
        ROS_VERSION="noetic"
    fi
    
    echo "安装ROS $ROS_VERSION..."
    
    # 添加ROS源
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # 添加密钥
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    
    # 更新并安装ROS
    sudo apt-get update
    sudo apt-get install -y ros-$ROS_VERSION-desktop-full
    
    # 初始化rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # 设置环境
    echo "source /opt/ros/$ROS_VERSION/setup.bash" >> ~/.bashrc
    source /opt/ros/$ROS_VERSION/setup.bash
    
    # 安装额外的ROS包
    sudo apt-get install -y \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-catkin-tools
        
    echo "ROS $ROS_VERSION 安装完成!"
else
    echo "检测到ROS $ROS_DISTRO 已安装"
fi

# 安装Python依赖
echo "安装Python依赖..."

# 创建requirements文件
cat > /tmp/requirements.txt << EOF
pymavlink>=2.4.37
matplotlib>=3.3.0
numpy>=1.19.0
scipy>=1.5.0
tf2-ros
tf2-geometry-msgs
geometry-msgs
sensor-msgs
nav-msgs
std-msgs
rospy
EOF

# 安装Python包
pip3 install --user -r /tmp/requirements.txt

# 安装系统级Python包
sudo apt-get install -y \
    python3-matplotlib \
    python3-numpy \
    python3-scipy \
    python3-tk \
    python3-pil \
    python3-pil.imagetk

# 安装ROS Python包 (如果ROS已安装)
if [ ! -z "$ROS_DISTRO" ]; then
    sudo apt-get install -y \
        ros-$ROS_DISTRO-geometry-msgs \
        ros-$ROS_DISTRO-sensor-msgs \
        ros-$ROS_DISTRO-nav-msgs \
        ros-$ROS_DISTRO-std-msgs \
        ros-$ROS_DISTRO-tf2-ros \
        ros-$ROS_DISTRO-tf2-geometry-msgs
fi

# 检查MAVProxy (可选)
echo "检查MAVProxy..."
if ! command -v mavproxy.py &> /dev/null; then
    echo "安装MAVProxy (可选，用于调试)..."
    pip3 install --user MAVProxy
fi

# 设置权限
echo "设置脚本权限..."
chmod +x *.py
chmod +x *.sh

# 创建桌面快捷方式 (可选)
echo "创建桌面快捷方式..."
DESKTOP_DIR="$HOME/Desktop"
if [ -d "$DESKTOP_DIR" ]; then
    cat > "$DESKTOP_DIR/ArduSub_Control.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=ArduSub Control System
Comment=ArduSub ROS Control System
Exec=gnome-terminal -- bash -c "cd $(pwd) && ./start_ardusub_system.sh"
Icon=applications-engineering
Terminal=true
Categories=Development;
EOF
    chmod +x "$DESKTOP_DIR/ArduSub_Control.desktop"
fi

# 验证安装
echo ""
echo "=================================="
echo "       验证安装"
echo "=================================="

# 检查Python模块
echo "检查Python模块..."
python3 -c "
import sys
modules = ['rospy', 'pymavlink', 'matplotlib', 'numpy', 'tkinter']
missing = []
for module in modules:
    try:
        __import__(module)
        print(f'✓ {module}')
    except ImportError:
        print(f'✗ {module} - 缺失')
        missing.append(module)

if missing:
    print(f'警告: 缺少模块: {missing}')
    sys.exit(1)
else:
    print('所有Python模块检查通过!')
"

# 检查ROS
if [ ! -z "$ROS_DISTRO" ]; then
    echo "✓ ROS $ROS_DISTRO"
else
    echo "✗ ROS 未安装或未设置环境"
fi

# 检查文件
echo "检查项目文件..."
files=(
    "ros_keyboard_control_node.py"
    "ardusub_status_ui.py"
    "ardusub_ros_interface_advanced.py"
    "start_ardusub_system.sh"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "✓ $file"
    else
        echo "✗ $file - 缺失"
    fi
done

echo ""
echo "=================================="
echo "       安装完成!"
echo "=================================="
echo ""
echo "使用方法:"
echo "1. 连接ArduSub到电脑"
echo "2. 运行: ./start_ardusub_system.sh"
echo "3. 选择启动模式"
echo ""
echo "注意事项:"
echo "- 首次运行前请重新登录以加载ROS环境"
echo "- 确保ArduSub通过MAVLink连接 (UDP 14551端口)"
echo "- 键盘控制需要终端焦点"
echo ""
echo "如有问题，请检查:"
echo "- ROS环境: echo \$ROS_DISTRO"
echo "- Python路径: which python3"
echo "- 端口占用: netstat -an | grep 14551"
echo ""

# 清理临时文件
rm -f /tmp/requirements.txt

echo "安装脚本执行完成!"
