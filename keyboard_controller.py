#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub 键盘控制节点
通过标准输入接收键盘输入，发布ROS话题控制ArduSub
避免使用keyboard库，不需要sudo权限
"""

import rospy
import sys
import select
import termios
import tty
import threading
import time

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Bool, String, Int32

class ArduSubKeyboardController:
    """ArduSub键盘控制器"""
    
    def __init__(self):
        rospy.init_node('ardusub_keyboard_controller', anonymous=True)
        rospy.loginfo("初始化ArduSub键盘控制器...")
        
        # 初始化发布者
        self.cmd_vel_pub = rospy.Publisher('/ardusub/cmd_vel', Twist, queue_size=1)
        self.arm_pub = rospy.Publisher('/ardusub/arm', Bool, queue_size=1)
        self.mode_pub = rospy.Publisher('/ardusub/set_mode', String, queue_size=1)
        self.lights_pub = rospy.Publisher('/ardusub/lights', Int32, queue_size=1)
        self.target_depth_pub = rospy.Publisher('/ardusub/target_depth', Float32, queue_size=1)
        self.gimbal_pub = rospy.Publisher('/ardusub/gimbal', Vector3, queue_size=1)
        self.servo_pub = rospy.Publisher('/ardusub/servo_control', Vector3, queue_size=1)
        
        # 控制状态
        self.current_twist = Twist()
        self.is_armed = False
        self.current_mode = "MANUAL"
        self.speed_scale = 0.5  # 速度缩放因子
        self.depth_step = 0.5   # 深度步长
        self.target_depth = 0.0
        
        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 控制指令映射
        self.key_bindings = {
            # 基本运动控制
            'w': ('forward', '前进'),
            's': ('backward', '后退'),
            'a': ('left', '左移'),
            'd': ('right', '右移'),
            'q': ('up', '上浮'),
            'e': ('down', '下潜'),
            'j': ('yaw_left', '左转'),
            'l': ('yaw_right', '右转'),
            
            # 速度调节
            '+': ('speed_up', '增加速度'),
            '-': ('speed_down', '减少速度'),
            
            # 深度控制
            'r': ('depth_up', '目标深度上升'),
            'f': ('depth_down', '目标深度下降'),
            
            # 系统控制
            'z': ('arm_toggle', '解锁/上锁切换'),
            'x': ('emergency_stop', '紧急停止'),
            
            # 模式切换
            '1': ('mode_manual', '手动模式'),
            '2': ('mode_stabilize', '稳定模式'),
            '3': ('mode_depth_hold', '定深模式'),
            
            # 灯光控制
            'i': ('light_main_on', '主灯亮'),
            'o': ('light_main_off', '主灯灭'),
            'k': ('light_ring_on', '光圈亮'),
            'm': ('light_ring_off', '光圈灭'),
            
            # 云台控制
            't': ('gimbal_up', '云台向上'),
            'g': ('gimbal_down', '云台向下'),
            'y': ('gimbal_center', '云台居中'),
            
            # 舵机控制
            'u': ('servo_1_up', '舵机1增加'),
            'h': ('servo_1_down', '舵机1减少'),
            
            # 系统
            ' ': ('stop', '停止所有运动'),
            'c': ('clear', '清屏'),
            'p': ('help', '显示帮助'),
            '\x1b': ('quit', '退出'),  # ESC键
        }
        
        rospy.loginfo("ArduSub键盘控制器初始化完成")
        
        # 显示帮助信息
        self.show_help()
    
    def show_help(self):
        """显示帮助信息"""
        print("\n" + "="*60)
        print("         ArduSub 键盘控制器")
        print("="*60)
        print("基本运动控制:")
        print("  W/S     - 前进/后退")
        print("  A/D     - 左移/右移") 
        print("  Q/E     - 上浮/下潜")
        print("  J/L     - 左转/右转")
        print("  空格    - 停止所有运动")
        print()
        print("速度控制:")
        print("  +/-     - 增加/减少速度 (当前: {:.1f})".format(self.speed_scale))
        print()
        print("深度控制:")
        print("  R/F     - 目标深度上升/下降 (当前: {:.1f}m)".format(self.target_depth))
        print()
        print("系统控制:")
        print("  Z       - 解锁/上锁切换")
        print("  X       - 紧急停止")
        print()
        print("模式切换:")
        print("  1/2/3   - 手动/稳定/定深模式")
        print()
        print("灯光控制:")
        print("  I/O     - 主灯亮/灭")
        print("  K/M     - 光圈亮/灭")
        print()
        print("云台控制:")
        print("  T/G     - 云台向上/向下")
        print("  Y       - 云台居中")
        print()
        print("舵机控制:")
        print("  U/H     - 舵机1增加/减少")
        print()
        print("其他:")
        print("  P       - 显示此帮助")
        print("  C       - 清屏")
        print("  ESC     - 退出程序")
        print("="*60)
        print("当前状态: 速度={:.1f}, 深度目标={:.1f}m".format(self.speed_scale, self.target_depth))
        print("按任意键开始控制...")
        print()
    
    def get_key(self):
        """获取键盘输入"""
        tty.setcraw(sys.stdin.fileno())
        try:
            # 检查是否有输入
            if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                key = sys.stdin.read(1)
                # 处理ESC键序列
                if key == '\x1b':
                    # 读取可能的额外字符（箭头键等）
                    if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                        key += sys.stdin.read(2)
                return key
            return ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def process_key(self, key):
        """处理键盘输入"""
        if not key:
            return True
        
        # 将大写字母转换为小写
        key = key.lower()
        
        if key in self.key_bindings:
            action, description = self.key_bindings[key]
            self.execute_action(action, description)
        else:
            print(f"未知按键: {repr(key)}")
        
        return key != '\x1b'  # ESC键退出
    
    def execute_action(self, action, description):
        """执行控制动作"""
        print(f"执行: {description}")
        
        # 基本运动控制
        if action == 'forward':
            self.current_twist.linear.x = self.speed_scale
            self.current_twist.linear.y = 0
        elif action == 'backward':
            self.current_twist.linear.x = -self.speed_scale
            self.current_twist.linear.y = 0
        elif action == 'left':
            self.current_twist.linear.y = self.speed_scale
            self.current_twist.linear.x = 0
        elif action == 'right':
            self.current_twist.linear.y = -self.speed_scale
            self.current_twist.linear.x = 0
        elif action == 'up':
            self.current_twist.linear.z = self.speed_scale
        elif action == 'down':
            self.current_twist.linear.z = -self.speed_scale
        elif action == 'yaw_left':
            self.current_twist.angular.z = self.speed_scale
        elif action == 'yaw_right':
            self.current_twist.angular.z = -self.speed_scale
        elif action == 'stop':
            self.current_twist = Twist()  # 重置所有运动
        
        # 速度调节
        elif action == 'speed_up':
            self.speed_scale = min(1.0, self.speed_scale + 0.1)
            print(f"速度增加到: {self.speed_scale:.1f}")
        elif action == 'speed_down':
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
            print(f"速度减少到: {self.speed_scale:.1f}")
        
        # 深度控制
        elif action == 'depth_up':
            self.target_depth = max(0.0, self.target_depth - self.depth_step)
            print(f"目标深度: {self.target_depth:.1f}m")
            depth_msg = Float32()
            depth_msg.data = self.target_depth
            self.target_depth_pub.publish(depth_msg)
        elif action == 'depth_down':
            self.target_depth += self.depth_step
            print(f"目标深度: {self.target_depth:.1f}m")
            depth_msg = Float32()
            depth_msg.data = self.target_depth
            self.target_depth_pub.publish(depth_msg)
        
        # 系统控制
        elif action == 'arm_toggle':
            self.is_armed = not self.is_armed
            arm_msg = Bool()
            arm_msg.data = self.is_armed
            self.arm_pub.publish(arm_msg)
            print(f"{'解锁' if self.is_armed else '上锁'}")
        elif action == 'emergency_stop':
            self.current_twist = Twist()
            arm_msg = Bool()
            arm_msg.data = False
            self.arm_pub.publish(arm_msg)
            print("紧急停止！已上锁并停止所有运动")
        
        # 模式切换
        elif action == 'mode_manual':
            self.current_mode = "MANUAL"
            mode_msg = String()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)
        elif action == 'mode_stabilize':
            self.current_mode = "STABILIZE"
            mode_msg = String()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)
        elif action == 'mode_depth_hold':
            self.current_mode = "DEPTH_HOLD"
            mode_msg = String()
            mode_msg.data = self.current_mode
            self.mode_pub.publish(mode_msg)
        
        # 灯光控制
        elif action == 'light_main_on':
            light_msg = Int32()
            light_msg.data = 1
            self.lights_pub.publish(light_msg)
        elif action == 'light_main_off':
            light_msg = Int32()
            light_msg.data = 0
            self.lights_pub.publish(light_msg)
        elif action == 'light_ring_on':
            light_msg = Int32()
            light_msg.data = 2
            self.lights_pub.publish(light_msg)
        elif action == 'light_ring_off':
            light_msg = Int32()
            light_msg.data = 3
            self.lights_pub.publish(light_msg)
        
        # 云台控制
        elif action == 'gimbal_up':
            gimbal_msg = Vector3()
            gimbal_msg.x = 30  # 向上30度
            self.gimbal_pub.publish(gimbal_msg)
        elif action == 'gimbal_down':
            gimbal_msg = Vector3()
            gimbal_msg.x = -30  # 向下30度
            self.gimbal_pub.publish(gimbal_msg)
        elif action == 'gimbal_center':
            gimbal_msg = Vector3()
            gimbal_msg.x = 0  # 水平
            self.gimbal_pub.publish(gimbal_msg)
        
        # 舵机控制
        elif action == 'servo_1_up':
            servo_msg = Vector3()
            servo_msg.x = 1  # 舵机ID
            servo_msg.y = 1700  # PWM值
            self.servo_pub.publish(servo_msg)
        elif action == 'servo_1_down':
            servo_msg = Vector3()
            servo_msg.x = 1  # 舵机ID
            servo_msg.y = 1300  # PWM值
            self.servo_pub.publish(servo_msg)
        
        # 其他
        elif action == 'clear':
            print("\033[2J\033[H")  # 清屏
            self.show_help()
        elif action == 'help':
            self.show_help()
        elif action == 'quit':
            print("退出程序...")
            return False
        
        # 发布运动控制指令（如果有运动）
        if action in ['forward', 'backward', 'left', 'right', 'up', 'down', 'yaw_left', 'yaw_right', 'stop']:
            self.cmd_vel_pub.publish(self.current_twist)
            # 显示当前运动状态
            if action != 'stop':
                print(f"运动状态: x={self.current_twist.linear.x:.1f}, y={self.current_twist.linear.y:.1f}, z={self.current_twist.linear.z:.1f}, yaw={self.current_twist.angular.z:.1f}")
        
        return True
    
    def status_publisher_thread(self):
        """状态发布线程 - 持续发布当前控制指令"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # 持续发布当前运动指令
            self.cmd_vel_pub.publish(self.current_twist)
            rate.sleep()
    
    def run(self):
        """运行控制循环"""
        try:
            # 启动状态发布线程
            status_thread = threading.Thread(target=self.status_publisher_thread)
            status_thread.daemon = True
            status_thread.start()
            
            print("开始键盘控制 (按ESC退出)...")
            
            while not rospy.is_shutdown():
                key = self.get_key()
                if not self.process_key(key):
                    break
                    
        except KeyboardInterrupt:
            print("\n键盘中断，退出程序...")
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # 停止所有运动
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            print("程序已退出")


def main():
    try:
        controller = ArduSubKeyboardController()
        controller.run()
    except rospy.ROSInterruptException:
        print("ROS节点被中断")
    except Exception as e:
        print(f"程序异常: {e}")


if __name__ == '__main__':
    main()
