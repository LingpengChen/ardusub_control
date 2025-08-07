# ArduSub ROS Control System

roslaunch ros1_oak_ffc_sync oak_ffc_sync.launch

/oak_ffc_sync_publisher/imu
/oak_ffc_sync_publisher/CAM_A/image
/oak_ffc_sync_publisher/CAM_B/image
/oak_ffc_sync_publisher/CAM_C/image
/oak_ffc_sync_publisher/CAM_D/image
/cameras/downward/image_raw



cd /home/liasorin-clp/workspace/ardusub_control

rosbag record \
-o bag/1.bag \
--lz4 \
/oak_ffc_sync_publisher/imu \
/oak_ffc_sync_publisher/CAM_A/image/compressed \
/oak_ffc_sync_publisher/CAM_B/image/compressed \
/oak_ffc_sync_publisher/CAM_C/image/compressed \
/oak_ffc_sync_publisher/CAM_D/image/compressed \
/cameras/downward/image_raw/compressed \
/oculus/drawn_sonar \
/oculus/drawn_sonar_osd \
/oculus/drawn_sonar_rect




roscore
/home/liasorin-clp/workspace/ardusub_control/start_cam.sh
/home/liasorin-clp/workspace/ardusub_control/start_sonar_and_cam.sh
rviz -d rviz/visualizer.rviz 



rosbag play -l bag/

这是一个基于ROS的ArduSub水下机器人控制系统，提供键盘控制、状态监控UI以及完整的ROS话题接口。

## 主要特性

- 🎮 **键盘控制**: 不依赖特殊库的键盘控制接口
- 📊 **状态监控UI**: 实时显示机器人姿态、深度、电池等状态
- 🔗 **ROS接口**: 完整的ROS话题接口，支持与其他ROS节点通信
- 🎯 **模块化设计**: 独立的ROS节点，可单独运行或组合使用
- 🚀 **易于使用**: 一键启动脚本，支持多种运行模式

## 系统架构

```
┌─────────────────┐    ┌─────────────────────┐    ┌──────────────────┐
│  键盘控制节点    │    │   ArduSub ROS接口    │    │   状态监控UI      │
│                │    │                     │    │                  │
│ - 键盘输入处理   │───▶│ - MAVLink通信       │───▶│ - 实时状态显示    │
│ - ROS话题发布   │    │ - 控制指令转换       │    │ - 历史数据图表    │
│ - 控制指令生成   │    │ - 传感器数据发布     │    │ - 系统监控       │
└─────────────────┘    └─────────────────────┘    └──────────────────┘
         │                       │                         │
         └───────────────────────┼─────────────────────────┘
                                 │
                       ┌─────────▼─────────┐
                       │   ROS Core        │
                       │   (话题通信)       │
                       └───────────────────┘
                                 │
                       ┌─────────▼─────────┐
                       │   ArduSub飞控     │
                       │   (MAVLink UDP)   │
                       └───────────────────┘
```

## ROS话题接口

### 发布的话题 (ArduSub → ROS)
- `/ardusub/imu` - IMU传感器数据
- `/ardusub/pressure` - 压力传感器数据  
- `/ardusub/current_depth` - 当前深度
- `/ardusub/battery` - 电池状态
- `/ardusub/odometry` - 里程计数据
- `/ardusub/pose` - 位姿信息
- `/ardusub/armed_status` - 解锁状态
- `/ardusub/current_mode` - 当前飞行模式
- `/ardusub/connection_status` - 连接状态

# ArduSub ROS Control System

这是一个基于ROS的ArduSub水下机器人控制系统，提供键盘控制、状态监控UI以及完整的ROS话题接口。

## 主要特性

- 🎮 **键盘控制**: 不依赖特殊库的键盘控制接口
- 📊 **状态监控UI**: 实时显示机器人姿态、深度、电池等状态
- 🔗 **ROS接口**: 完整的ROS话题接口，支持与其他ROS节点通信
- 🎯 **模块化设计**: 独立的ROS节点，可单独运行或组合使用
- 🚀 **易于使用**: 一键启动脚本，支持多种运行模式

## 系统架构

```
┌─────────────────┐    ┌─────────────────────┐    ┌──────────────────┐
│  键盘控制节点    │    │   ArduSub ROS接口    │    │   状态监控UI      │
│                │    │                     │    │                  │
│ - 键盘输入处理   │───▶│ - MAVLink通信       │───▶│ - 实时状态显示    │
│ - ROS话题发布   │    │ - 控制指令转换       │    │ - 历史数据图表    │
│ - 控制指令生成   │    │ - 传感器数据发布     │    │ - 系统监控       │
└─────────────────┘    └─────────────────────┘    └──────────────────┘
         │                       │                         │
         └───────────────────────┼─────────────────────────┘
                                 │
                       ┌─────────▼─────────┐
                       │   ROS Core        │
                       │   (话题通信)       │
                       └───────────────────┘
                                 │
                       ┌─────────▼─────────┐
                       │   ArduSub飞控     │
                       │   (MAVLink UDP)   │
                       └───────────────────┘
```

## ROS话题接口

### 发布的话题 (ArduSub → ROS)
- `/ardusub/imu` - IMU传感器数据
- `/ardusub/pressure` - 压力传感器数据  
- `/ardusub/current_depth` - 当前深度
- `/ardusub/battery` - 电池状态
- `/ardusub/odometry` - 里程计数据
- `/ardusub/pose` - 位姿信息
- `/ardusub/armed_status` - 解锁状态
- `/ardusub/current_mode` - 当前飞行模式
- `/ardusub/connection_status` - 连接状态

### 订阅的话题 (ROS → ArduSub)
- `/ardusub/cmd_vel` - 速度控制指令
- `/ardusub/arm` - 解锁/上锁指令
- `/ardusub/set_mode` - 飞行模式设置
- `/ardusub/lights` - 灯光控制
- `/ardusub/target_depth` - 目标深度设置
- `/ardusub/gimbal_control` - 云台控制
- `/ardusub/servo_control` - 舵机控制

## 快速开始

### 1. 安装依赖

```bash
# 使用提供的安装脚本
chmod +x install_ros_dependencies.sh
./install_ros_dependencies.sh

# 或手动安装
sudo apt-get install python3-tk python3-matplotlib ros-noetic-desktop-full
pip3 install pymavlink matplotlib numpy
```

### 2. 连接ArduSub

确保ArduSub通过MAVLink连接到电脑：
- 默认端口：UDP 14551
- 连接字符串：`udpin:0.0.0.0:14551`

### 3. 启动系统

```bash
# 一键启动（推荐）
./start_ardusub_system.sh

# 或分别启动各组件
python3 ardusub_ros_interface_advanced.py &  # ArduSub接口
python3 ardusub_status_ui.py &               # 状态监控UI  
python3 ros_keyboard_control_node.py         # 键盘控制
```

## 使用说明

### 键盘控制

| 按键 | 功能 | 按键 | 功能 |
|------|------|------|------|
| W/S | 前进/后退 | A/D | 左移/右移 |
| Q/E | 上浮/下潜 | J/L | 左转/右转 |
| I/K | 抬头/低头 | +/- | 增减速度 |
| R/F | 目标深度上升/下降 | 空格 | 停止所有运动 |
| Z | 解锁/上锁切换 | X | 紧急停止 |
| 1/2/3/4 | 手动/稳定/定深/定点模式 | O/P | 增减灯光亮度 |
| T/G | 云台向上/向下 | Y | 云台居中 |
| H | 显示帮助 | C | 清屏 |
| ESC | 退出程序 | | |

### 状态监控UI

状态监控UI提供以下信息：
- **系统状态**: 电机解锁状态、飞行模式、连接状态
- **位置姿态**: 当前深度、目标深度、Roll/Pitch/Yaw角度
- **控制状态**: 各轴推力输出值
- **设备状态**: 灯光亮度、云台角度、电池电压
- **历史图表**: 深度变化曲线、姿态变化曲线

## 项目文件说明

### 核心程序
- `ardusub_ros_interface_advanced.py` - ArduSub ROS接口（MAVLink通信）
- `ros_keyboard_control_node.py` - 键盘控制ROS节点
- `ardusub_status_ui.py` - 状态监控图形界面

### 启动脚本
- `start_ardusub_system.sh` - 系统启动脚本
- `install_ros_dependencies.sh` - 依赖安装脚本

### 原有文件
- `control_ardusub.py` - 原始ArduSub控制库
- `ros_ardusub_interface.py` - 简化版ROS接口
- `keyboard_controller.py` - 原始键盘控制器

## 系统要求

- **操作系统**: Ubuntu 18.04/20.04 LTS
- **ROS版本**: Melodic/Noetic
- **Python**: 3.6+
- **依赖包**: 
  - ROS桌面完整版
  - Python: pymavlink, matplotlib, numpy, tkinter
  - 系统: python3-tk, python3-matplotlib

## 故障排除

### 常见问题

1. **ROS环境未设置**
   ```bash
   source /opt/ros/noetic/setup.bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   ```

2. **ArduSub连接失败**
   - 检查UDP端口14551是否被占用
   - 确认ArduSub MAVLink输出设置正确
   - 检查防火墙设置

3. **键盘控制无响应**
   - 确保终端窗口有焦点
   - 检查ROS节点是否正常运行: `rosnode list`

4. **UI显示异常**
   - 检查是否安装了tkinter: `python3 -c "import tkinter"`
   - 安装matplotlib: `pip3 install matplotlib`

### 调试命令

```bash
# 查看ROS节点
rosnode list

# 查看话题
rostopic list

# 监控话题数据
rostopic echo /ardusub/imu

# 检查MAVLink连接
mavproxy.py --master=udp:127.0.0.1:14551
```

## 扩展开发

### 自定义控制节点

可以创建自己的ROS节点来发布控制指令：

```python
import rospy
from geometry_msgs.msg import Twist

# 初始化节点
rospy.init_node('custom_controller')
cmd_pub = rospy.Publisher('/ardusub/cmd_vel', Twist, queue_size=1)

# 发送控制指令
cmd = Twist()
cmd.linear.x = 500  # 前进
cmd_pub.publish(cmd)
```

### 添加新的传感器

在`ardusub_ros_interface_advanced.py`中添加新的MAVLink消息处理：

```python
def process_new_sensor(self, msg):
    # 处理新传感器数据
    sensor_msg = SensorMsg()
    # ... 填充数据
    self.new_sensor_pub.publish(sensor_msg)
```

## 贡献

欢迎提交Issue和Pull Request来改进这个项目！

## 许可证

MIT License

## 联系方式

如有问题请创建Issue或联系开发者。

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
