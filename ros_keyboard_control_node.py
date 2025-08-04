#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub ROS键盘控制节点
不使用keyboard库，通过标准输入获取键盘输入
发布ROS话题控制ArduSub
"""

import rospy
import sys
import select
import termios
import tty
import threading
import time
import os

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import Imu, FluidPressure
from nav_msgs.msg import Odometry

class ArduSubKeyboardControlNode:
    """ArduSub键盘控制ROS节点"""
    
    def __init__(self):
        rospy.init_node('ardusub_keyboard_control', anonymous=True)
        rospy.loginfo("启动ArduSub键盘控制节点...")
        
        # 初始化发布者
        self.setup_publishers()
        
        # 初始化订阅者（用于获取状态信息）
        self.setup_subscribers()
        
        # 控制状态
        self.current_twist = Twist()
        self.is_armed = False
        self.current_mode = "MANUAL"
        self.speed_scale = 500  # PWM速度缩放因子 (1100-1900范围)
        self.depth_step = 0.5   # 深度步长(米)
        self.target_depth = 0.0
        self.light_brightness = 0  # 灯光亮度 0-100
        self.gimbal_tilt = 0  # 云台俯仰角度
        
        # 机器人状态信息
        self.robot_depth = 0.0
        self.robot_orientation = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.robot_position = {'x': 0, 'y': 0, 'z': 0}
        self.battery_voltage = 0.0
        
        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 控制映射
        self.setup_key_bindings()
        
        # 状态更新定时器
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.publish_control_commands)
        
        rospy.loginfo("ArduSub键盘控制节点初始化完成")
        
    def setup_publishers(self):
        """设置ROS发布者"""
        self.cmd_vel_pub = rospy.Publisher('/ardusub/cmd_vel', Twist, queue_size=1)
        self.arm_pub = rospy.Publisher('/ardusub/arm', Bool, queue_size=1)
        self.mode_pub = rospy.Publisher('/ardusub/set_mode', String, queue_size=1)
        self.lights_pub = rospy.Publisher('/ardusub/lights', Int32, queue_size=1)
        self.target_depth_pub = rospy.Publisher('/ardusub/target_depth', Float32, queue_size=1)
        self.gimbal_pub = rospy.Publisher('/ardusub/gimbal_control', Vector3, queue_size=1)
        self.servo_pub = rospy.Publisher('/ardusub/servo_control', Vector3, queue_size=1)
        
    def setup_subscribers(self):
        """设置ROS订阅者"""
        rospy.Subscriber('/ardusub/imu', Imu, self.imu_callback)
        rospy.Subscriber('/ardusub/pressure', FluidPressure, self.pressure_callback)
        rospy.Subscriber('/ardusub/odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('/ardusub/status', String, self.status_callback)
        
    def setup_key_bindings(self):
        """设置键盘映射"""
        self.key_bindings = {
            # 基本运动控制 (WASD + QE)
            'w': self.move_forward,
            's': self.move_backward,
            'a': self.move_left,
            'd': self.move_right,
            'q': self.move_up,
            'e': self.move_down,
            
            # 旋转控制
            'j': self.rotate_left,
            'l': self.rotate_right,
            'i': self.rotate_up,    # pitch up
            'k': self.rotate_down,  # pitch down
            
            # 速度调节
            '+': self.increase_speed,
            '=': self.increase_speed,  # 不按shift的+
            '-': self.decrease_speed,
            
            # 深度控制
            'r': self.depth_up,
            'f': self.depth_down,
            
            # 系统控制
            'z': self.toggle_arm,
            'x': self.emergency_stop,
            
            # 模式切换
            '1': lambda: self.set_mode('MANUAL'),
            '2': lambda: self.set_mode('STABILIZE'),
            '3': lambda: self.set_mode('DEPTH_HOLD'),
            '4': lambda: self.set_mode('POSHOLD'),
            
            # 灯光控制
            'o': self.lights_brighter,
            'p': self.lights_dimmer,
            
            # 云台控制
            't': self.gimbal_up,
            'g': self.gimbal_down,
            'y': self.gimbal_center,
            
            # 停止和帮助
            ' ': self.stop_all,
            'h': self.show_help,
            'c': self.clear_screen,
        }
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        # 从四元数转换为欧拉角
        import tf.transformations
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.robot_orientation = {
            'roll': euler[0] * 180.0 / 3.14159,
            'pitch': euler[1] * 180.0 / 3.14159,
            'yaw': euler[2] * 180.0 / 3.14159
        }
    
    def pressure_callback(self, msg):
        """压力传感器数据回调"""
        # 根据压力计算深度 (假设1bar = 10m水深)
        self.robot_depth = (msg.fluid_pressure - 101325) / 9806.65
    
    def odometry_callback(self, msg):
        """里程计数据回调"""
        self.robot_position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
    
    def status_callback(self, msg):
        """状态数据回调"""
        # 解析状态信息
        pass
    
    def move_forward(self):
        self.current_twist.linear.x = min(self.current_twist.linear.x + self.speed_scale, self.speed_scale)
        self.log_action("前进")
    
    def move_backward(self):
        self.current_twist.linear.x = max(self.current_twist.linear.x - self.speed_scale, -self.speed_scale)
        self.log_action("后退")
    
    def move_left(self):
        self.current_twist.linear.y = min(self.current_twist.linear.y + self.speed_scale, self.speed_scale)
        self.log_action("左移")
    
    def move_right(self):
        self.current_twist.linear.y = max(self.current_twist.linear.y - self.speed_scale, -self.speed_scale)
        self.log_action("右移")
    
    def move_up(self):
        self.current_twist.linear.z = min(self.current_twist.linear.z + self.speed_scale, self.speed_scale)
        self.log_action("上浮")
    
    def move_down(self):
        self.current_twist.linear.z = max(self.current_twist.linear.z - self.speed_scale, -self.speed_scale)
        self.log_action("下潜")
    
    def rotate_left(self):
        self.current_twist.angular.z = min(self.current_twist.angular.z + self.speed_scale, self.speed_scale)
        self.log_action("左转")
    
    def rotate_right(self):
        self.current_twist.angular.z = max(self.current_twist.angular.z - self.speed_scale, -self.speed_scale)
        self.log_action("右转")
    
    def rotate_up(self):
        self.current_twist.angular.y = min(self.current_twist.angular.y + self.speed_scale, self.speed_scale)
        self.log_action("抬头")
    
    def rotate_down(self):
        self.current_twist.angular.y = max(self.current_twist.angular.y - self.speed_scale, -self.speed_scale)
        self.log_action("低头")
    
    def increase_speed(self):
        self.speed_scale = min(self.speed_scale + 50, 800)
        self.log_action(f"增加速度至 {self.speed_scale}")
    
    def decrease_speed(self):
        self.speed_scale = max(self.speed_scale - 50, 100)
        self.log_action(f"减少速度至 {self.speed_scale}")
    
    def depth_up(self):
        self.target_depth -= self.depth_step
        self.target_depth_pub.publish(Float32(data=self.target_depth))
        self.log_action(f"目标深度: {self.target_depth:.1f}m")
    
    def depth_down(self):
        self.target_depth += self.depth_step
        self.target_depth_pub.publish(Float32(data=self.target_depth))
        self.log_action(f"目标深度: {self.target_depth:.1f}m")
    
    def toggle_arm(self):
        self.is_armed = not self.is_armed
        self.arm_pub.publish(Bool(data=self.is_armed))
        status = "解锁" if self.is_armed else "上锁"
        self.log_action(f"电机{status}")
    
    def emergency_stop(self):
        self.current_twist = Twist()  # 清零所有运动
        self.is_armed = False
        self.arm_pub.publish(Bool(data=False))
        self.log_action("紧急停止！")
    
    def set_mode(self, mode):
        self.current_mode = mode
        self.mode_pub.publish(String(data=mode))
        self.log_action(f"切换模式: {mode}")
    
    def lights_brighter(self):
        self.light_brightness = min(self.light_brightness + 10, 100)
        self.lights_pub.publish(Int32(data=self.light_brightness))
        self.log_action(f"灯光亮度: {self.light_brightness}%")
    
    def lights_dimmer(self):
        self.light_brightness = max(self.light_brightness - 10, 0)
        self.lights_pub.publish(Int32(data=self.light_brightness))
        self.log_action(f"灯光亮度: {self.light_brightness}%")
    
    def gimbal_up(self):
        self.gimbal_tilt = min(self.gimbal_tilt + 10, 90)
        gimbal_cmd = Vector3(x=0, y=self.gimbal_tilt, z=0)
        self.gimbal_pub.publish(gimbal_cmd)
        self.log_action(f"云台向上: {self.gimbal_tilt}°")
    
    def gimbal_down(self):
        self.gimbal_tilt = max(self.gimbal_tilt - 10, -90)
        gimbal_cmd = Vector3(x=0, y=self.gimbal_tilt, z=0)
        self.gimbal_pub.publish(gimbal_cmd)
        self.log_action(f"云台向下: {self.gimbal_tilt}°")
    
    def gimbal_center(self):
        self.gimbal_tilt = 0
        gimbal_cmd = Vector3(x=0, y=0, z=0)
        self.gimbal_pub.publish(gimbal_cmd)
        self.log_action("云台居中")
    
    def stop_all(self):
        self.current_twist = Twist()
        self.log_action("停止所有运动")
    
    def clear_screen(self):
        os.system('clear')
        self.show_status()
    
    def show_help(self):
        """显示帮助信息"""
        help_text = """
╔══════════════════════════════════════════════════════════════╗
║                    ArduSub 键盘控制帮助                        ║
╠══════════════════════════════════════════════════════════════╣
║ 基本运动控制:                                                ║
║   W/S    - 前进/后退        A/D    - 左移/右移                ║
║   Q/E    - 上浮/下潜        J/L    - 左转/右转                ║
║   I/K    - 抬头/低头                                          ║
║                                                              ║
║ 速度控制:                                                    ║
║   +/-    - 增加/减少速度                                      ║
║                                                              ║
║ 深度控制:                                                    ║
║   R/F    - 目标深度上升/下降                                  ║
║                                                              ║
║ 系统控制:                                                    ║
║   Z      - 解锁/上锁切换    X      - 紧急停止                ║
║                                                              ║
║ 模式切换:                                                    ║
║   1/2/3/4 - 手动/稳定/定深/定点模式                          ║
║                                                              ║
║ 灯光控制:                                                    ║
║   O/P    - 增加/减少灯光亮度                                  ║
║                                                              ║
║ 云台控制:                                                    ║
║   T/G    - 云台向上/向下    Y      - 云台居中                ║
║                                                              ║
║ 其他:                                                        ║
║   空格   - 停止所有运动      H      - 显示帮助                ║
║   C      - 清屏             ESC/Ctrl+C - 退出              ║
╚══════════════════════════════════════════════════════════════╝
        """
        print(help_text)
    
    def show_status(self):
        """显示状态信息"""
        status_text = f"""
╔══════════════════════════════════════════════════════════════╗
║                    ArduSub 状态信息                           ║
╠══════════════════════════════════════════════════════════════╣
║ 系统状态: {'解锁' if self.is_armed else '上锁':>8} | 模式: {self.current_mode:>12} ║
║ 速度缩放: {self.speed_scale:>8} | 深度目标: {self.target_depth:>8.1f}m ║
║ 当前深度: {self.robot_depth:>8.1f}m | 灯光亮度: {self.light_brightness:>6}% ║
║                                                              ║
║ 姿态信息:                                                    ║
║   Roll:  {self.robot_orientation['roll']:>8.1f}° | Pitch: {self.robot_orientation['pitch']:>8.1f}° ║
║   Yaw:   {self.robot_orientation['yaw']:>8.1f}° | 云台:   {self.gimbal_tilt:>8.1f}° ║
║                                                              ║
║ 运动控制:                                                    ║
║   前后: {self.current_twist.linear.x:>8.0f} | 左右: {self.current_twist.linear.y:>8.0f} ║
║   升降: {self.current_twist.linear.z:>8.0f} | 转向: {self.current_twist.angular.z:>8.0f} ║
╚══════════════════════════════════════════════════════════════╝
        """
        print(status_text)
    
    def log_action(self, action):
        """记录动作"""
        timestamp = rospy.Time.now().to_sec()
        rospy.loginfo(f"[{timestamp:.1f}] {action}")
    
    def publish_control_commands(self, event):
        """定时发布控制指令"""
        self.cmd_vel_pub.publish(self.current_twist)
    
    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        """主运行循环"""
        self.clear_screen()
        self.show_help()
        self.show_status()
        
        print("\n按 'H' 显示帮助, 按 'ESC' 或 Ctrl+C 退出")
        print("开始键盘控制...")
        
        try:
            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = self.get_key()
                    
                    # 处理ESC键退出
                    if key == '\x1b':  # ESC
                        break
                    
                    # 处理Ctrl+C
                    if key == '\x03':  # Ctrl+C
                        break
                    
                    # 执行对应的控制函数
                    if key in self.key_bindings:
                        self.key_bindings[key]()
                    
                    # 更新状态显示
                    if key in ['h', 'c']:
                        pass  # 这些键已经处理了显示
                    else:
                        # 移动光标到底部显示当前状态
                        print(f"\r当前控制: 前后:{self.current_twist.linear.x:4.0f} 左右:{self.current_twist.linear.y:4.0f} 上下:{self.current_twist.linear.z:4.0f} 旋转:{self.current_twist.angular.z:4.0f} | 按H查看帮助", end='', flush=True)
                
                # 检查ROS节点状态
                if rospy.is_shutdown():
                    break
                    
        except KeyboardInterrupt:
            pass
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # 发送停止指令
            self.emergency_stop()
            print("\n键盘控制节点已退出")

def main():
    try:
        controller = ArduSubKeyboardControlNode()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"键盘控制节点错误: {e}")

if __name__ == '__main__':
    main()
