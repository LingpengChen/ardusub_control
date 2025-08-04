#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub ROS键盘控制节点 - Hold-to-Move版本
按住按键才移动，松开按键自动停止
"""

import rospy
import sys
import select
import termios
import tty
import threading
import time
import os
from pynput import keyboard

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import Imu, FluidPressure
from nav_msgs.msg import Odometry

class ArduSubHoldToMoveController:
    """ArduSub按住移动控制器"""
    
    def __init__(self):
        rospy.init_node('ardusub_hold_to_move_control', anonymous=True)
        rospy.loginfo("启动ArduSub按住移动控制节点...")
        
        # 初始化发布者
        self.setup_publishers()
        
        # 初始化订阅者（用于获取状态信息）
        self.setup_subscribers()
        
        # 控制状态
        self.current_twist = Twist()
        self.is_armed = False
        self.current_mode = "MANUAL"
        self.speed_scale = 500  # PWM速度缩放因子
        self.depth_step = 0.5   # 深度步长(米)
        self.target_depth = 0.0
        self.light_brightness = 0  # 灯光亮度 0-100
        self.gimbal_tilt = 0  # 云台俯仰角度
        
        # 按键状态跟踪
        self.pressed_keys = set()
        self.key_actions = {
            'w': ('linear', 'x', 1),    # 前进
            's': ('linear', 'x', -1),   # 后退
            'a': ('linear', 'y', 1),    # 左移
            'd': ('linear', 'y', -1),   # 右移
            'q': ('linear', 'z', 1),    # 上浮
            'e': ('linear', 'z', -1),   # 下潜
            'j': ('angular', 'z', 1),   # 左转
            'l': ('angular', 'z', -1),  # 右转
            'i': ('angular', 'y', 1),   # 抬头
            'k': ('angular', 'y', -1),  # 低头
        }
        
        # 机器人状态信息
        self.robot_depth = 0.0
        self.robot_orientation = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.robot_position = {'x': 0, 'y': 0, 'z': 0}
        self.battery_voltage = 0.0
        
        # # 保存终端设置
        # self.settings = termios.tcgetattr(sys.stdin)
        
        # # 启动控制更新线程
        # self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        # self.control_thread.start()

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()
        
        # 状态更新定时器
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.publish_control_commands)
        
        rospy.loginfo("ArduSub按住移动控制节点初始化完成")
        
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
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        try:
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
        except ImportError:
            import math
            x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp)
            else:
                pitch = math.asin(sinp)
            
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            self.robot_orientation = {
                'roll': roll * 180.0 / math.pi,
                'pitch': pitch * 180.0 / math.pi,
                'yaw': yaw * 180.0 / math.pi
            }
    
    def pressure_callback(self, msg):
        """压力传感器数据回调"""
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
        pass
    
    # def get_key_non_blocking(self):
    #     """非阻塞获取键盘输入"""
    #     if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
    #         return sys.stdin.read(1)
    #     return None
    

    def on_press(self, key):
        """pynput 按键按下时的回调函数"""
        try:
            # 尝试获取按键的字符形式
            char = key.char.lower()
        except AttributeError:
            # 如果是特殊按键（如Esc），则没有.char属性
            if key == keyboard.Key.esc:
                rospy.signal_shutdown("User pressed ESC")
                self.listener.stop() # 停止监听器
            return

        # 避免重复触发（按住一个键时，系统会重复发送按下事件）
        if char in self.pressed_keys:
            return

        # 处理运动按键
        if char in self.key_actions:
            self.pressed_keys.add(char)
            self.update_movement()
            print(f"\r[按下] {char.upper()} - {self.get_action_name(char)}", end='', flush=True)

        # 处理单次触发的命令
        elif char == 'z':
            self.toggle_arm()
        elif char == 'x' or char == ' ':
            self.emergency_stop()
        elif char == 'h':
            self.show_help()
        elif char == 'c':
            self.clear_screen()
        elif char == '+' or char == '=':
            self.increase_speed()
        elif char == '-':
            self.decrease_speed()
        elif char == 'r':
            self.depth_up()
        elif char == 'f':
            self.depth_down()
        elif char == 'o':
            self.lights_brighter()
        elif char == 'p':
            self.lights_dimmer()
        elif char == 't':
            self.gimbal_up()
        elif char == 'g':
            self.gimbal_down()
        elif char == 'y':
            self.gimbal_center()
        elif char in '1234':
            modes = {'1': 'MANUAL', '2': 'STABILIZE', '3': 'DEPTH_HOLD', '4': 'POSHOLD'}
            self.set_mode(modes[char])
            
    def on_release(self, key):
        """pynput 按键松开时的回调函数"""
        try:
            char = key.char.lower()
        except AttributeError:
            # 处理特殊按键的释放
            if key == keyboard.Key.esc:
                return
            return

        # 如果松开的是一个正在被按下的运动键
        if char in self.pressed_keys:
            self.pressed_keys.remove(char)
            self.update_movement()
            print(f"\n[松开] {char.upper()} - 停止 {self.get_action_name(char)}")
            self.show_status() # 松开按键后可以更新一下状态显示

    def control_loop(self):
        """控制循环 - 持续检测按键状态"""
        # 设置终端为非缓冲模式
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while not rospy.is_shutdown():
                # 检查是否有新的按键输入
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    # 处理特殊键
                    if key == '\x1b':  # ESC
                        rospy.signal_shutdown("User pressed ESC")
                        break
                    elif key == '\x03':  # Ctrl+C
                        rospy.signal_shutdown("User pressed Ctrl+C")
                        break
                    elif key == ' ':  # 空格 - 停止所有运动
                        self.emergency_stop()
                    elif key == 'z':  # 切换解锁
                        self.toggle_arm()
                    elif key == 'x':  # 紧急停止
                        self.emergency_stop()
                    elif key == 'h':  # 显示帮助
                        self.show_help()
                    elif key == 'c':  # 清屏
                        self.clear_screen()
                    elif key == '+' or key == '=':
                        self.increase_speed()
                    elif key == '-':
                        self.decrease_speed()
                    elif key == 'r':
                        self.depth_up()
                    elif key == 'f':
                        self.depth_down()
                    elif key == 'o':
                        self.lights_brighter()
                    elif key == 'p':
                        self.lights_dimmer()
                    elif key == 't':
                        self.gimbal_up()
                    elif key == 'g':
                        self.gimbal_down()
                    elif key == 'y':
                        self.gimbal_center()
                    elif key == '1':
                        self.set_mode('MANUAL')
                    elif key == '2':
                        self.set_mode('STABILIZE')
                    elif key == '3':
                        self.set_mode('DEPTH_HOLD')
                    elif key == '4':
                        self.set_mode('POSHOLD')
                    
                    # 处理运动按键 - 按住才移动
                    if key.lower() in self.key_actions:
                        if key.lower() not in self.pressed_keys:
                            self.pressed_keys.add(key.lower())
                            self.update_movement()
                            print(f"\r[按住] {key.upper()} - {self.get_action_name(key.lower())}", end='', flush=True)
                
                # 更新运动状态（检测按键释放）
                self.update_movement()
                
                time.sleep(0.05)  # 20Hz更新率
                
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def get_action_name(self, key):
        """获取按键对应的动作名称"""
        action_names = {
            'w': '前进', 's': '后退', 'a': '左移', 'd': '右移',
            'q': '上浮', 'e': '下潜', 'j': '左转', 'l': '右转',
            'i': '抬头', 'k': '低头'
        }
        return action_names.get(key, '未知')
    
    def update_movement(self):
        """根据当前按下的按键更新运动指令"""
        # 重置所有运动
        self.current_twist = Twist()
        
        # 根据按下的按键设置运动
        for key in self.pressed_keys.copy():
            if key in self.key_actions:
                axis_type, axis, direction = self.key_actions[key]
                value = direction * self.speed_scale
                
                if axis_type == 'linear':
                    if axis == 'x':
                        self.current_twist.linear.x = value
                    elif axis == 'y':
                        self.current_twist.linear.y = value
                    elif axis == 'z':
                        self.current_twist.linear.z = value
                elif axis_type == 'angular':
                    if axis == 'y':
                        self.current_twist.angular.y = value
                    elif axis == 'z':
                        self.current_twist.angular.z = value
        
        # 如果没有按键被按下，确保所有运动都停止
        if not self.pressed_keys:
            self.current_twist = Twist()
    
    def toggle_arm(self):
        """切换解锁状态"""
        self.is_armed = not self.is_armed
        self.arm_pub.publish(Bool(data=self.is_armed))
        status = "解锁" if self.is_armed else "上锁"
        print(f"\n电机{status}")
        self.log_action(f"电机{status}")
    
    def emergency_stop(self):
        """紧急停止"""
        self.pressed_keys.clear()
        self.current_twist = Twist()
        self.is_armed = False
        self.arm_pub.publish(Bool(data=False))
        print("\n紧急停止！")
        self.log_action("紧急停止！")
    
    def set_mode(self, mode):
        """设置飞行模式"""
        self.current_mode = mode
        self.mode_pub.publish(String(data=mode))
        print(f"\n切换模式: {mode}")
        self.log_action(f"切换模式: {mode}")
    
    def increase_speed(self):
        """增加速度"""
        self.speed_scale = min(self.speed_scale + 50, 800)
        print(f"\n速度增加至: {self.speed_scale}")
        self.log_action(f"增加速度至 {self.speed_scale}")
    
    def decrease_speed(self):
        """减少速度"""
        self.speed_scale = max(self.speed_scale - 50, 100)
        print(f"\n速度减少至: {self.speed_scale}")
        self.log_action(f"减少速度至 {self.speed_scale}")
    
    def depth_up(self):
        """目标深度上升"""
        self.target_depth -= self.depth_step
        self.target_depth_pub.publish(Float32(data=self.target_depth))
        print(f"\n目标深度: {self.target_depth:.1f}m")
        self.log_action(f"目标深度: {self.target_depth:.1f}m")
    
    def depth_down(self):
        """目标深度下降"""
        self.target_depth += self.depth_step
        self.target_depth_pub.publish(Float32(data=self.target_depth))
        print(f"\n目标深度: {self.target_depth:.1f}m")
        self.log_action(f"目标深度: {self.target_depth:.1f}m")
    
    def lights_brighter(self):
        """增加灯光亮度"""
        # 发送灯光变亮指令到ROS接口
        # 不再直接管理亮度值，而是发送增量指令
        self.lights_pub.publish(Int32(data=1))  # 1表示变亮
        print(f"\n灯光变亮")
        self.log_action(f"灯光变亮")
    
    def lights_dimmer(self):
        """减少灯光亮度"""
        # 发送灯光变暗指令到ROS接口
        # 不再直接管理亮度值，而是发送减量指令
        self.lights_pub.publish(Int32(data=-1))  # -1表示变暗
        print(f"\n灯光变暗")
        self.log_action(f"灯光变暗")
    
    def gimbal_up(self):
        """云台向上"""
        self.gimbal_tilt = min(self.gimbal_tilt + 10, 90)
        gimbal_cmd = Vector3(x=0, y=self.gimbal_tilt, z=0)
        self.gimbal_pub.publish(gimbal_cmd)
        print(f"\n云台向上: {self.gimbal_tilt}°")
        self.log_action(f"云台向上: {self.gimbal_tilt}°")
    
    def gimbal_down(self):
        """云台向下"""
        self.gimbal_tilt = max(self.gimbal_tilt - 10, -90)
        gimbal_cmd = Vector3(x=0, y=self.gimbal_tilt, z=0)
        self.gimbal_pub.publish(gimbal_cmd)
        print(f"\n云台向下: {self.gimbal_tilt}°")
        self.log_action(f"云台向下: {self.gimbal_tilt}°")
    
    def gimbal_center(self):
        """云台居中"""
        self.gimbal_tilt = 0
        gimbal_cmd = Vector3(x=0, y=0, z=0)
        self.gimbal_pub.publish(gimbal_cmd)
        print(f"\n云台居中")
        self.log_action("云台居中")
    
    def clear_screen(self):
        """清屏"""
        os.system('clear')
        self.show_status()
    
    def show_help(self):
        """显示帮助信息"""
        help_text = """
╔══════════════════════════════════════════════════════════════╗
║                ArduSub 按住移动控制帮助                       ║
╠══════════════════════════════════════════════════════════════╣
║ 运动控制 (按住按键才移动，松开自动停止):                      ║
║   W/S    - 前进/后退        A/D    - 左移/右移                ║
║   Q/E    - 上浮/下潜        J/L    - 左转/右转                ║
║   I/K    - 抬头/低头                                          ║
║                                                              ║
║ 系统控制:                                                    ║
║   Z      - 解锁/上锁切换    X      - 紧急停止                ║
║   空格   - 停止所有运动                                       ║
║                                                              ║
║ 速度控制:                                                    ║
║   +/-    - 增加/减少速度                                      ║
║                                                              ║
║ 深度控制:                                                    ║
║   R/F    - 目标深度上升/下降                                  ║
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
║   H      - 显示帮助        C      - 清屏                     ║
║   ESC/Ctrl+C - 退出                                         ║
╚══════════════════════════════════════════════════════════════╝
        """
        print(help_text)
    
    def show_status(self):
        """显示状态信息"""
        pressed_keys_str = ', '.join(self.pressed_keys) if self.pressed_keys else '无'
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
║                                                              ║
║ 当前按键: {pressed_keys_str:<20}                              ║
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
    
    # 修改 run 方法
    def run(self):
        """主运行循环"""
        self.clear_screen()
        self.show_help()
        self.show_status()
        
        print("\n使用 pynput 实时控制机器人 (按H查看帮助)")
        print("按住移动键来控制，松开自动停止。按ESC退出。")
        
        # rospy.spin()会阻塞主线程，直到节点被关闭
        # 这能确保监听器线程和ROS回调函数能持续工作
        rospy.spin()

def main():
    controller = None
    try:
        controller = ArduSubHoldToMoveController()
        controller.run()
    except rospy.ROSInterruptException:
        print("\nROS 中断，节点退出。")
    except Exception as e:
        rospy.logerr(f"按住移动控制节点错误: {e}")
    finally:
        if controller and controller.listener.is_alive():
            controller.listener.stop()
        # 发送最后的停止指令
        if controller:
            controller.emergency_stop()
        print("\n按住移动控制节点已退出。")

if __name__ == '__main__':
    main()
