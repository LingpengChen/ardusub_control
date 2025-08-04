#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub ROS接口测试脚本
演示如何通过ROS话题控制ArduSub和获取状态数据
"""

import rospy
import time
import math
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import Imu, Temperature, FluidPressure
from nav_msgs.msg import Odometry

class ArduSubROSTest:
    """ArduSub ROS接口测试类"""
    
    def __init__(self):
        rospy.init_node('ardusub_test', anonymous=True)
        
        # 初始化发布者（用于发送控制指令）
        self.cmd_vel_pub = rospy.Publisher('/ardusub/cmd_vel', Twist, queue_size=10)
        self.thrust_x_pub = rospy.Publisher('/ardusub/thrust_x', Float32, queue_size=10)
        self.thrust_y_pub = rospy.Publisher('/ardusub/thrust_y', Float32, queue_size=10)
        self.thrust_z_pub = rospy.Publisher('/ardusub/thrust_z', Float32, queue_size=10)
        self.thrust_yaw_pub = rospy.Publisher('/ardusub/thrust_yaw', Float32, queue_size=10)
        self.target_depth_pub = rospy.Publisher('/ardusub/target_depth', Float32, queue_size=10)
        self.target_attitude_pub = rospy.Publisher('/ardusub/target_attitude', Vector3, queue_size=10)
        self.servo_pub = rospy.Publisher('/ardusub/servo_control', Vector3, queue_size=10)
        self.gimbal_pub = rospy.Publisher('/ardusub/gimbal', Vector3, queue_size=10)
        self.lights_pub = rospy.Publisher('/ardusub/lights', Int32, queue_size=10)
        self.arm_pub = rospy.Publisher('/ardusub/arm', Bool, queue_size=10)
        self.mode_pub = rospy.Publisher('/ardusub/set_mode', String, queue_size=10)
        
        # 初始化订阅者（用于接收状态数据）
        self.imu_sub = rospy.Subscriber('/ardusub/imu', Imu, self.imu_callback)
        self.depth_sub = rospy.Subscriber('/ardusub/depth', Float32, self.depth_callback)
        self.temp_sub = rospy.Subscriber('/ardusub/temperature', Temperature, self.temp_callback)
        self.pressure_sub = rospy.Subscriber('/ardusub/pressure', FluidPressure, self.pressure_callback)
        self.armed_sub = rospy.Subscriber('/ardusub/armed', Bool, self.armed_callback)
        self.odom_sub = rospy.Subscriber('/ardusub/odom', Odometry, self.odom_callback)
        
        # 状态变量
        self.current_depth = 0.0
        self.current_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.current_temperature = 0.0
        self.current_pressure = 0.0
        self.is_armed = False
        
        rospy.loginfo("ArduSub测试节点初始化完成")
        
        # 等待连接建立
        time.sleep(2)
    
    # ===== 状态数据回调函数 =====
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        # 将四元数转换为欧拉角（简化版本）
        # 这里只是示例，实际应用中可能需要更精确的转换
        self.current_attitude['roll'] = math.degrees(math.atan2(
            2.0 * (msg.orientation.w * msg.orientation.x + msg.orientation.y * msg.orientation.z),
            1.0 - 2.0 * (msg.orientation.x * msg.orientation.x + msg.orientation.y * msg.orientation.y)
        ))
        
        self.current_attitude['pitch'] = math.degrees(math.asin(
            2.0 * (msg.orientation.w * msg.orientation.y - msg.orientation.z * msg.orientation.x)
        ))
        
        self.current_attitude['yaw'] = math.degrees(math.atan2(
            2.0 * (msg.orientation.w * msg.orientation.z + msg.orientation.x * msg.orientation.y),
            1.0 - 2.0 * (msg.orientation.y * msg.orientation.y + msg.orientation.z * msg.orientation.z)
        ))
    
    def depth_callback(self, msg):
        """深度数据回调"""
        self.current_depth = msg.data
    
    def temp_callback(self, msg):
        """温度数据回调"""
        self.current_temperature = msg.temperature
    
    def pressure_callback(self, msg):
        """压力数据回调"""
        self.current_pressure = msg.fluid_pressure
    
    def armed_callback(self, msg):
        """解锁状态回调"""
        self.is_armed = msg.data
    
    def odom_callback(self, msg):
        """里程计数据回调"""
        # 可以在这里处理完整的位置和姿态信息
        pass
    
    # ===== 测试函数 =====
    
    def print_status(self):
        """打印当前状态"""
        print(f"\\n===== ArduSub 状态 =====")
        print(f"解锁状态: {'已解锁' if self.is_armed else '已上锁'}")
        print(f"深度: {self.current_depth:.2f} m")
        print(f"姿态: Roll={self.current_attitude['roll']:.1f}°, "
              f"Pitch={self.current_attitude['pitch']:.1f}°, "
              f"Yaw={self.current_attitude['yaw']:.1f}°")
        print(f"温度: {self.current_temperature:.1f}°C")
        print(f"压力: {self.current_pressure:.1f} Pa")
        print("========================")
    
    def test_arm_disarm(self):
        """测试解锁和上锁"""
        print("\\n测试解锁/上锁功能...")
        
        # 解锁
        print("发送解锁指令...")
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)
        time.sleep(2)
        
        # 上锁
        print("发送上锁指令...")
        arm_msg.data = False
        self.arm_pub.publish(arm_msg)
        time.sleep(2)
    
    def test_mode_switch(self):
        """测试模式切换"""
        print("\\n测试模式切换...")
        
        modes = ["MANUAL", "STABILIZE", "DEPTH_HOLD"]
        
        for mode in modes:
            print(f"切换到{mode}模式...")
            mode_msg = String()
            mode_msg.data = mode
            self.mode_pub.publish(mode_msg)
            time.sleep(2)
    
    def test_movement_control(self):
        """测试运动控制"""
        print("\\n测试运动控制...")
        
        # 使用Twist消息进行综合控制
        print("测试Twist控制 - 前进...")
        twist = Twist()
        twist.linear.x = 0.5  # 前进
        self.cmd_vel_pub.publish(twist)
        time.sleep(2)
        
        print("测试Twist控制 - 左移...")
        twist.linear.x = 0.0
        twist.linear.y = 0.5  # 左移
        self.cmd_vel_pub.publish(twist)
        time.sleep(2)
        
        print("测试Twist控制 - 上浮...")
        twist.linear.y = 0.0
        twist.linear.z = 0.3  # 上浮
        self.cmd_vel_pub.publish(twist)
        time.sleep(2)
        
        print("测试Twist控制 - 偏航...")
        twist.linear.z = 0.0
        twist.angular.z = 0.3  # 偏航
        self.cmd_vel_pub.publish(twist)
        time.sleep(2)
        
        # 停止
        print("停止运动...")
        twist = Twist()  # 所有值为0
        self.cmd_vel_pub.publish(twist)
        time.sleep(1)
        
        # 测试单独轴控制
        print("测试单独轴控制...")
        
        # X轴推力
        print("X轴推力测试...")
        thrust_msg = Float32()
        thrust_msg.data = 0.3
        self.thrust_x_pub.publish(thrust_msg)
        time.sleep(2)
        
        thrust_msg.data = 0.0
        self.thrust_x_pub.publish(thrust_msg)
        time.sleep(1)
    
    def test_depth_control(self):
        """测试深度控制"""
        print("\\n测试深度控制...")
        
        target_depths = [1.0, 2.0, 0.5, 0.0]
        
        for depth in target_depths:
            print(f"设置目标深度: {depth}m")
            depth_msg = Float32()
            depth_msg.data = depth
            self.target_depth_pub.publish(depth_msg)
            time.sleep(3)
    
    def test_attitude_control(self):
        """测试姿态控制"""
        print("\\n测试姿态控制...")
        
        attitudes = [
            (10, 0, 0),    # Roll 10度
            (0, 10, 0),    # Pitch 10度
            (0, 0, 45),    # Yaw 45度
            (0, 0, 0),     # 回到水平
        ]
        
        for roll, pitch, yaw in attitudes:
            print(f"设置目标姿态: Roll={roll}°, Pitch={pitch}°, Yaw={yaw}°")
            attitude_msg = Vector3()
            attitude_msg.x = roll
            attitude_msg.y = pitch
            attitude_msg.z = yaw
            self.target_attitude_pub.publish(attitude_msg)
            time.sleep(3)
    
    def test_lights_control(self):
        """测试灯光控制"""
        print("\\n测试灯光控制...")
        
        light_commands = [
            (1, "主灯亮"),
            (0, "主灯灭"),
            (2, "光圈亮"),
            (3, "光圈灭"),
        ]
        
        for cmd, desc in light_commands:
            print(f"{desc}...")
            light_msg = Int32()
            light_msg.data = cmd
            self.lights_pub.publish(light_msg)
            time.sleep(2)
    
    def test_gimbal_control(self):
        """测试云台控制"""
        print("\\n测试云台控制...")
        
        gimbal_positions = [
            (-30, 0, 0),   # 向下30度
            (0, 0, 0),     # 水平
            (30, 0, 0),    # 向上30度
            (0, 0, 0),     # 回到水平
        ]
        
        for tilt, roll, pan in gimbal_positions:
            print(f"云台位置: Tilt={tilt}°, Roll={roll}°, Pan={pan}°")
            gimbal_msg = Vector3()
            gimbal_msg.x = tilt
            gimbal_msg.y = roll
            gimbal_msg.z = pan
            self.gimbal_pub.publish(gimbal_msg)
            time.sleep(2)
    
    def test_servo_control(self):
        """测试舵机控制"""
        print("\\n测试舵机控制...")
        
        servo_commands = [
            (1, 1500),  # 舵机1，中位
            (1, 1200),  # 舵机1，最小
            (1, 1800),  # 舵机1，最大
            (1, 1500),  # 舵机1，回到中位
        ]
        
        for servo_id, pwm in servo_commands:
            print(f"舵机{servo_id}: PWM={pwm}")
            servo_msg = Vector3()
            servo_msg.x = servo_id
            servo_msg.y = pwm
            servo_msg.z = 0
            self.servo_pub.publish(servo_msg)
            time.sleep(2)
    
    def run_all_tests(self):
        """运行所有测试"""
        print("开始ArduSub ROS接口测试...")
        
        # 等待系统稳定
        time.sleep(3)
        
        try:
            # 显示初始状态
            self.print_status()
            
            # 运行各项测试
            self.test_arm_disarm()
            self.test_mode_switch()
            self.test_movement_control()
            self.test_lights_control()
            self.test_gimbal_control()
            self.test_servo_control()
            
            # 如果需要测试深度和姿态控制，请取消下面的注释
            # 注意：这些功能需要ArduSub处于相应的模式下
            # self.test_depth_control()
            # self.test_attitude_control()
            
            print("\\n所有测试完成！")
            
        except Exception as e:
            print(f"测试过程中出现错误: {e}")
    
    def interactive_mode(self):
        """交互模式"""
        print("\\n进入交互模式...")
        print("可用命令:")
        print("  status - 显示状态")
        print("  arm - 解锁")
        print("  disarm - 上锁")
        print("  mode <模式> - 切换模式 (MANUAL/STABILIZE/DEPTH_HOLD)")
        print("  move <x> <y> <z> <yaw> - 运动控制")
        print("  depth <深度> - 设置目标深度")
        print("  lights <命令> - 灯光控制 (0-3)")
        print("  quit - 退出")
        print()
        
        while not rospy.is_shutdown():
            try:
                cmd = input("请输入命令: ").strip().split()
                if not cmd:
                    continue
                
                if cmd[0] == "quit":
                    break
                elif cmd[0] == "status":
                    self.print_status()
                elif cmd[0] == "arm":
                    arm_msg = Bool()
                    arm_msg.data = True
                    self.arm_pub.publish(arm_msg)
                    print("发送解锁指令")
                elif cmd[0] == "disarm":
                    arm_msg = Bool()
                    arm_msg.data = False
                    self.arm_pub.publish(arm_msg)
                    print("发送上锁指令")
                elif cmd[0] == "mode" and len(cmd) > 1:
                    mode_msg = String()
                    mode_msg.data = cmd[1].upper()
                    self.mode_pub.publish(mode_msg)
                    print(f"切换到{cmd[1].upper()}模式")
                elif cmd[0] == "move" and len(cmd) >= 5:
                    twist = Twist()
                    twist.linear.x = float(cmd[1])
                    twist.linear.y = float(cmd[2])
                    twist.linear.z = float(cmd[3])
                    twist.angular.z = float(cmd[4])
                    self.cmd_vel_pub.publish(twist)
                    print(f"运动控制: x={cmd[1]}, y={cmd[2]}, z={cmd[3]}, yaw={cmd[4]}")
                elif cmd[0] == "depth" and len(cmd) > 1:
                    depth_msg = Float32()
                    depth_msg.data = float(cmd[1])
                    self.target_depth_pub.publish(depth_msg)
                    print(f"设置目标深度: {cmd[1]}m")
                elif cmd[0] == "lights" and len(cmd) > 1:
                    light_msg = Int32()
                    light_msg.data = int(cmd[1])
                    self.lights_pub.publish(light_msg)
                    print(f"灯光控制: {cmd[1]}")
                else:
                    print("无效命令或参数不足")
            
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"命令执行错误: {e}")
        
        print("退出交互模式")


if __name__ == '__main__':
    try:
        test = ArduSubROSTest()
        
        print("ArduSub ROS接口测试")
        print("1. 运行自动测试")
        print("2. 进入交互模式")
        print("3. 仅监控状态")
        
        choice = input("请选择模式 (1/2/3): ").strip()
        
        if choice == "1":
            test.run_all_tests()
        elif choice == "2":
            test.interactive_mode()
        elif choice == "3":
            print("监控模式 - 按Ctrl+C退出")
            rate = rospy.Rate(1)  # 1Hz
            while not rospy.is_shutdown():
                test.print_status()
                rate.sleep()
        else:
            print("无效选择")
            
    except rospy.ROSInterruptException:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序异常退出: {e}")
