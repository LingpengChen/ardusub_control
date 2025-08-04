# ArduSub ROS 接口

这是一个简化的ArduSub ROS接口，用于通过ROS话题与ArduSub飞控进行通信和控制。该接口避免了复杂的ROS包结构，直接使用Python3脚本和Shell脚本来实现。

## 功能特点

- **简化部署**: 不需要catkin工作空间和复杂的ROS包结构
- **Python3原生**: 直接使用python3运行，无需rosrun
- **完整功能**: 支持ArduSub的所有主要控制功能
- **实时数据**: 发布飞控状态数据到ROS话题
- **易于扩展**: 模块化设计，便于添加新功能

## 系统要求

- Ubuntu 18.04/20.04/22.04
- ROS Melodic/Noetic
- Python 3.6+
- pymavlink

## 安装依赖

```bash
# 安装ROS (如果还没有安装)
sudo apt update
sudo apt install ros-noetic-desktop-full  # 或 ros-melodic-desktop-full

# 安装Python依赖
pip3 install pymavlink

# 设置ROS环境 (添加到 ~/.bashrc)
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 文件说明

- `ros_ardusub_interface.py` - 主要的ROS接口程序
- `start_ardusub_ros.sh` - 启动脚本
- `test_ardusub_ros.py` - 测试脚本
- `simple_controller.py` - 简单使用示例
- `control_ardusub.py` - 原始的ArduSub控制代码
- `udp_connect.py` - 原始的UDP通信代码
- `connect_main.py` - 原始的主程序

## 快速开始

### 1. 启动ArduSub ROS接口

```bash
# 给启动脚本添加执行权限
chmod +x start_ardusub_ros.sh

# 启动接口 (默认连接地址: udpin:0.0.0.0:14551)
./start_ardusub_ros.sh

# 或指定自定义连接地址
ARDUSUB_CONNECTION="udpin:192.168.1.100:14551" ./start_ardusub_ros.sh
```

### 2. 运行测试程序

```bash
# 在另一个终端运行测试
python3 test_ardusub_ros.py
```

### 3. 运行简单控制示例

```bash
python3 simple_controller.py
```

## ROS话题接口

### 发布的话题 (ArduSub状态数据)

| 话题名称 | 消息类型 | 描述 |
|---------|----------|------|
| `/ardusub/imu` | `sensor_msgs/Imu` | IMU数据（姿态角、角速度等） |
| `/ardusub/depth` | `std_msgs/Float32` | 当前深度 (米) |
| `/ardusub/temperature` | `sensor_msgs/Temperature` | 水温 (摄氏度) |
| `/ardusub/pressure` | `sensor_msgs/FluidPressure` | 水压 (帕斯卡) |
| `/ardusub/armed` | `std_msgs/Bool` | 解锁状态 |
| `/ardusub/odom` | `nav_msgs/Odometry` | 里程计数据（位置+姿态） |

### 订阅的话题 (控制指令)

| 话题名称 | 消息类型 | 描述 |
|---------|----------|------|
| `/ardusub/cmd_vel` | `geometry_msgs/Twist` | 速度控制（前后、左右、上下、偏航） |
| `/ardusub/thrust_x` | `std_msgs/Float32` | X轴推力 (-1.0 ~ 1.0) |
| `/ardusub/thrust_y` | `std_msgs/Float32` | Y轴推力 (-1.0 ~ 1.0) |
| `/ardusub/thrust_z` | `std_msgs/Float32` | Z轴推力 (-1.0 ~ 1.0) |
| `/ardusub/thrust_yaw` | `std_msgs/Float32` | 偏航推力 (-1.0 ~ 1.0) |
| `/ardusub/target_depth` | `std_msgs/Float32` | 目标深度 (米) |
| `/ardusub/target_attitude` | `geometry_msgs/Vector3` | 目标姿态 (度) |
| `/ardusub/servo_control` | `geometry_msgs/Vector3` | 舵机控制 (x=ID, y=PWM, z=未使用) |
| `/ardusub/gimbal` | `geometry_msgs/Vector3` | 云台控制 (x=俯仰, y=横滚, z=偏航) |
| `/ardusub/lights` | `std_msgs/Int32` | 灯光控制 (0=主灯灭, 1=主灯亮, 2=光圈亮, 3=光圈灭) |
| `/ardusub/arm` | `std_msgs/Bool` | 解锁/上锁控制 |
| `/ardusub/set_mode` | `std_msgs/String` | 模式切换 (MANUAL/STABILIZE/DEPTH_HOLD) |

## 使用示例

### 基本控制命令

```bash
# 解锁ArduSub
rostopic pub /ardusub/arm std_msgs/Bool "data: true"

# 前进 (x=0.5表示50%推力)
rostopic pub /ardusub/cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}"

# 上浮
rostopic pub /ardusub/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.3}"

# 左转
rostopic pub /ardusub/cmd_vel geometry_msgs/Twist "angular: {z: 0.3}"

# 停止所有运动
rostopic pub /ardusub/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}"

# 切换到稳定模式
rostopic pub /ardusub/set_mode std_msgs/String "data: 'STABILIZE'"

# 设置目标深度2米
rostopic pub /ardusub/target_depth std_msgs/Float32 "data: 2.0"

# 主灯亮
rostopic pub /ardusub/lights std_msgs/Int32 "data: 1"

# 上锁
rostopic pub /ardusub/arm std_msgs/Bool "data: false"
```

### 查看状态数据

```bash
# 查看当前深度
rostopic echo /ardusub/depth

# 查看IMU数据
rostopic echo /ardusub/imu

# 查看解锁状态
rostopic echo /ardusub/armed

# 列出所有可用话题
rostopic list
```

### Python代码示例

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

# 初始化节点
rospy.init_node('my_ardusub_controller')

# 创建发布者
cmd_vel_pub = rospy.Publisher('/ardusub/cmd_vel', Twist, queue_size=10)
arm_pub = rospy.Publisher('/ardusub/arm', Bool, queue_size=10)

# 等待连接建立
rospy.sleep(1)

# 解锁
arm_msg = Bool()
arm_msg.data = True
arm_pub.publish(arm_msg)

# 前进2秒
twist = Twist()
twist.linear.x = 0.5
for _ in range(20):  # 10Hz * 2秒 = 20次
    cmd_vel_pub.publish(twist)
    rospy.sleep(0.1)

# 停止
twist.linear.x = 0.0
cmd_vel_pub.publish(twist)
```

## 故障排除

### 1. MAVLink连接失败
- 检查ArduSub是否正在运行
- 确认连接地址和端口正确
- 检查防火墙设置

### 2. ROS话题无数据
- 确认roscore正在运行
- 检查节点是否正常启动
- 使用`rostopic list`确认话题存在

### 3. 控制指令不响应
- 确认ArduSub已解锁
- 检查飞行模式是否正确
- 确认MAVLink连接正常

### 4. 权限问题
- 确保脚本有执行权限：`chmod +x *.sh`
- 避免使用sudo运行，这可能导致ROS环境问题

## 高级配置

### 自定义连接参数

编辑`ros_ardusub_interface.py`中的连接参数：

```python
# 默认连接字符串
self.connection_string = rospy.get_param('~connection_string', 'udpin:0.0.0.0:14551')

# 数据更新频率
self.data_rate = rospy.get_param('~data_rate', 20)  # Hz
```

### 添加自定义按钮功能

在`setup_button_functions()`方法中添加：

```python
# 添加新的按钮配置
self.set_button_function(15, 'custom_function')
```

### 扩展消息类型

可以根据需要添加更多ROS消息类型和话题，例如：

- 声纳数据发布
- GPS位置信息
- 电池状态
- 自定义传感器数据

## 贡献

欢迎提交问题报告和功能请求。如果要贡献代码，请：

1. Fork这个项目
2. 创建功能分支
3. 提交更改
4. 发起Pull Request

## 许可证

该项目使用MIT许可证。详见LICENSE文件。

## 联系方式

如有问题或建议，请创建GitHub Issue。
