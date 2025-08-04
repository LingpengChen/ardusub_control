#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS ArduSub Interface - 简化版本不使用rospackage
通过ROS话题接收控制指令，发送给ArduSub飞控
同时发布飞控的状态数据
"""

import rospy
import sys
import os
import time
import math
import threading

# ROS消息类型
from std_msgs.msg import Header, Float32, Int32, Bool
from geometry_msgs.msg import Twist, Vector3, Quaternion
from sensor_msgs.msg import Imu, FluidPressure, Temperature

# 自定义消息 - 简化版本使用标准消息类型组合
from nav_msgs.msg import Odometry

# MAVLink相关
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

class ArduSubROSInterface:
    """ArduSub ROS接口类"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('ardusub_ros_interface', anonymous=True)
        rospy.loginfo("初始化ArduSub ROS接口...")
        
        # 连接参数
        self.connection_string = rospy.get_param('~connection_string', 'udpin:0.0.0.0:14551')
        self.data_rate = rospy.get_param('~data_rate', 20)  # Hz
        
        # 初始化MAVLink连接
        self.init_mavlink_connection()
        
        # 状态变量
        self.boot_time = time.time()
        self.armed = False
        self.current_mode = "UNKNOWN"
        
        # 控制变量
        self.x = 0  # 前进后退
        self.y = 0  # 左右平移
        self.z = 500  # 上下(油门)
        self.r = 0  # 偏航
        
        # 传感器数据
        self.attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
        self.depth = 0.0
        self.temperature = 0.0
        self.pressure = 0.0
        
        # 初始化ROS发布者
        self.init_publishers()
        
        # 初始化ROS订阅者
        self.init_subscribers()
        
        # 配置按钮功能
        self.setup_button_functions()
        
        # 启动数据接收线程
        self.start_data_thread()
        
        rospy.loginfo("ArduSub ROS接口初始化完成")
    
    def init_mavlink_connection(self):
        """初始化MAVLink连接"""
        try:
            rospy.loginfo(f"连接到ArduSub: {self.connection_string}")
            self.master = mavutil.mavlink_connection(self.connection_string)
            self.master.wait_heartbeat()
            rospy.loginfo("MAVLink连接成功!")
            
            # 请求数据流
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                self.data_rate,
                1
            )
            
        except Exception as e:
            rospy.logerr(f"MAVLink连接失败: {e}")
            sys.exit(1)
    
    def init_publishers(self):
        """初始化ROS发布者"""
        # 姿态数据
        self.imu_pub = rospy.Publisher('/ardusub/imu', Imu, queue_size=10)
        
        # 深度和高度
        self.depth_pub = rospy.Publisher('/ardusub/depth', Float32, queue_size=10)
        
        # 温度
        self.temp_pub = rospy.Publisher('/ardusub/temperature', Temperature, queue_size=10)
        
        # 压力
        self.pressure_pub = rospy.Publisher('/ardusub/pressure', FluidPressure, queue_size=10)
        
        # 状态信息
        self.armed_pub = rospy.Publisher('/ardusub/armed', Bool, queue_size=10)
        self.mode_pub = rospy.Publisher('/ardusub/mode', std_msgs.msg.String, queue_size=10)
        
        # 里程计信息（位置+姿态）
        self.odom_pub = rospy.Publisher('/ardusub/odom', Odometry, queue_size=10)
        
    def init_subscribers(self):
        """初始化ROS订阅者"""
        # 速度控制（Twist消息）
        self.cmd_vel_sub = rospy.Subscriber('/ardusub/cmd_vel', Twist, self.cmd_vel_callback)
        
        # 单独的推进器控制
        self.thrust_x_sub = rospy.Subscriber('/ardusub/thrust_x', Float32, self.thrust_x_callback)
        self.thrust_y_sub = rospy.Subscriber('/ardusub/thrust_y', Float32, self.thrust_y_callback)
        self.thrust_z_sub = rospy.Subscriber('/ardusub/thrust_z', Float32, self.thrust_z_callback)
        self.thrust_yaw_sub = rospy.Subscriber('/ardusub/thrust_yaw', Float32, self.thrust_yaw_callback)
        
        # 目标深度设置
        self.target_depth_sub = rospy.Subscriber('/ardusub/target_depth', Float32, self.target_depth_callback)
        
        # 目标姿态设置
        self.target_attitude_sub = rospy.Subscriber('/ardusub/target_attitude', Vector3, self.target_attitude_callback)
        
        # 舵机控制
        self.servo_sub = rospy.Subscriber('/ardusub/servo_control', Vector3, self.servo_callback)
        
        # 云台控制
        self.gimbal_sub = rospy.Subscriber('/ardusub/gimbal', Vector3, self.gimbal_callback)
        
        # 灯光控制
        self.lights_sub = rospy.Subscriber('/ardusub/lights', Int32, self.lights_callback)
        
        # 解锁/上锁
        self.arm_sub = rospy.Subscriber('/ardusub/arm', Bool, self.arm_callback)
        
        # 模式切换
        self.mode_sub = rospy.Subscriber('/ardusub/set_mode', std_msgs.msg.String, self.mode_callback)
    
    def setup_button_functions(self):
        """设置按钮功能"""
        rospy.loginfo("配置按钮功能...")
        
        button_configs = [
            (10, 'lights2_brighter'),
            (9, 'lights2_dimmer'),
            (13, 'servo_2_min'),
            (14, 'servo_2_max'),
            (0, 'servo_3_min'),
            (3, 'servo_3_max'),
            (1, 'servo_1_inc'),
            (2, 'servo_1_dec'),
            (11, 'mount_tilt_up'),
            (12, 'mount_tilt_down'),
            (8, 'input_hold_set'),
            (5, 'shift'),
            (7, 'shift'),
        ]
        
        shifted_configs = [
            (0, 'mode_manual'),
            (1, 'mode_stabilize'),
            (3, 'mode_depth_hold'),
            (2, 'mount_center'),
        ]
        
        try:
            for button, function in button_configs:
                self.set_button_function(button, function)
            
            for button, function in shifted_configs:
                self.set_button_function(button, function, shifted=True)
                
            rospy.loginfo("按钮功能配置完成")
        except Exception as e:
            rospy.logwarn(f"按钮功能配置失败: {e}")
    
    def set_button_function(self, button, function, shifted=False):
        """设置按钮功能"""
        ARDUSUB_BTN_FUNCTIONS = {
            'disabled': 0, 'shift': 1, 'arm_toggle': 2, 'arm': 3, 'disarm': 4,
            'mode_manual': 5, 'mode_stabilize': 6, 'mode_depth_hold': 7,
            'mode_poshold': 8, 'mode_auto': 9, 'mount_center': 21,
            'mount_tilt_up': 22, 'mount_tilt_down': 23, 'lights1_brighter': 32,
            'lights1_dimmer': 33, 'lights2_brighter': 35, 'lights2_dimmer': 36,
            'gain_inc': 42, 'gain_dec': 43, 'input_hold_set': 48,
            'servo_1_inc': 61, 'servo_1_dec': 62, 'servo_2_min': 68,
            'servo_2_max': 69, 'servo_3_min': 73, 'servo_3_max': 74,
        }
        
        shifted_str = 'S' if shifted else ''
        param = f'BTN{button}_{shifted_str}FUNCTION'
        
        if isinstance(function, str):
            function = ARDUSUB_BTN_FUNCTIONS.get(function.lower(), 0)
        
        param_bytes = param.encode('utf8')
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            param_bytes,
            function,
            mavutil.mavlink.MAV_PARAM_TYPE_INT8
        )
    
    def start_data_thread(self):
        """启动数据接收线程"""
        self.data_thread = threading.Thread(target=self.data_receiver_thread)
        self.data_thread.daemon = True
        self.data_thread.start()
        
        rospy.loginfo("数据接收线程已启动")
    
    def data_receiver_thread(self):
        """数据接收线程"""
        rate = rospy.Rate(50)  # 50Hz
        
        while not rospy.is_shutdown():
            try:
                msg = self.master.recv_match(blocking=False, timeout=0.1)
                
                if msg is not None:
                    self.process_mavlink_message(msg)
                    
                # 发布ROS消息
                self.publish_ros_messages()
                
            except Exception as e:
                rospy.logwarn(f"数据接收错误: {e}")
            
            rate.sleep()
    
    def process_mavlink_message(self, msg):
        """处理MAVLink消息"""
        msg_type = msg.get_type()
        
        if msg_type == 'ATTITUDE':
            self.attitude['roll'] = msg.roll
            self.attitude['pitch'] = msg.pitch
            self.attitude['yaw'] = msg.yaw
            
        elif msg_type == 'VFR_HUD':
            self.depth = -msg.alt  # ArduSub中高度为负值表示深度
            
        elif msg_type == 'SCALED_PRESSURE2':
            self.temperature = msg.temperature / 100.0  # 转换为摄氏度
            self.pressure = msg.press_abs * 100.0  # 转换为Pa
            
        elif msg_type == 'HEARTBEAT':
            self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            
        elif msg_type == 'SYS_STATUS':
            pass  # 可以添加系统状态处理
    
    def publish_ros_messages(self):
        """发布ROS消息"""
        current_time = rospy.Time.now()
        
        # 发布IMU数据
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = "base_link"
        
        # 转换欧拉角到四元数
        q = QuaternionBase([self.attitude['roll'], self.attitude['pitch'], self.attitude['yaw']])
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        
        self.imu_pub.publish(imu_msg)
        
        # 发布深度
        depth_msg = Float32()
        depth_msg.data = self.depth
        self.depth_pub.publish(depth_msg)
        
        # 发布温度
        temp_msg = Temperature()
        temp_msg.header.stamp = current_time
        temp_msg.temperature = self.temperature
        self.temp_pub.publish(temp_msg)
        
        # 发布压力
        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = current_time
        pressure_msg.fluid_pressure = self.pressure
        self.pressure_pub.publish(pressure_msg)
        
        # 发布解锁状态
        armed_msg = Bool()
        armed_msg.data = self.armed
        self.armed_pub.publish(armed_msg)
        
        # 发布里程计信息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # 位置（只有深度有效）
        odom_msg.pose.pose.position.z = -self.depth
        
        # 姿态
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        self.odom_pub.publish(odom_msg)
    
    # ===== ROS回调函数 =====
    
    def cmd_vel_callback(self, msg):
        """速度控制回调"""
        # 将Twist消息转换为ArduSub控制量
        self.x = int(msg.linear.x * 1000)  # 前进后退
        self.y = int(msg.linear.y * 1000)  # 左右平移
        self.z = int(msg.linear.z * 1000 + 500)  # 上下（加上中位值）
        self.r = int(msg.angular.z * 1000)  # 偏航
        
        # 限制范围
        self.x = max(-2000, min(2000, self.x))
        self.y = max(-2000, min(2000, self.y))
        self.z = max(-500, min(1500, self.z))
        self.r = max(-2000, min(2000, self.r))
        
        self.send_manual_control()
    
    def thrust_x_callback(self, msg):
        """X轴推力控制"""
        self.x = int(msg.data * 1000)
        self.x = max(-2000, min(2000, self.x))
        self.send_manual_control()
    
    def thrust_y_callback(self, msg):
        """Y轴推力控制"""
        self.y = int(msg.data * 1000)
        self.y = max(-2000, min(2000, self.y))
        self.send_manual_control()
    
    def thrust_z_callback(self, msg):
        """Z轴推力控制"""
        self.z = int(msg.data * 1000 + 500)
        self.z = max(-500, min(1500, self.z))
        self.send_manual_control()
    
    def thrust_yaw_callback(self, msg):
        """偏航推力控制"""
        self.r = int(msg.data * 1000)
        self.r = max(-2000, min(2000, self.r))
        self.send_manual_control()
    
    def target_depth_callback(self, msg):
        """目标深度设置"""
        self.set_target_depth(msg.data)
    
    def target_attitude_callback(self, msg):
        """目标姿态设置"""
        self.set_target_attitude(msg.x, msg.y, msg.z)
    
    def servo_callback(self, msg):
        """舵机控制"""
        servo_id = int(msg.x)
        pwm_value = int(msg.y)
        self.set_servo_pwm(servo_id, pwm_value)
    
    def gimbal_callback(self, msg):
        """云台控制"""
        self.look_at(msg.x, msg.y, msg.z)
    
    def lights_callback(self, msg):
        """灯光控制"""
        if msg.data == 1:  # 主灯亮
            self.press_release_buttons([14])
        elif msg.data == 0:  # 主灯灭
            self.press_release_buttons([13])
        elif msg.data == 2:  # 光圈亮
            self.press_release_buttons([10])
        elif msg.data == 3:  # 光圈灭
            self.press_release_buttons([9])
    
    def arm_callback(self, msg):
        """解锁/上锁控制"""
        if msg.data:
            self.master.arducopter_arm()
            rospy.loginfo("ArduSub已解锁")
        else:
            self.master.arducopter_disarm()
            rospy.loginfo("ArduSub已上锁")
    
    def mode_callback(self, msg):
        """模式切换"""
        mode = msg.data.upper()
        if mode == "MANUAL":
            self.press_release_buttons([0, 5])
        elif mode == "STABILIZE":
            self.press_release_buttons([1, 5])
        elif mode == "DEPTH_HOLD":
            self.press_release_buttons([3, 5])
        
        rospy.loginfo(f"切换到{mode}模式")
    
    # ===== MAVLink控制函数 =====
    
    def send_manual_control(self):
        """发送手动控制指令"""
        self.master.mav.manual_control_send(
            self.master.target_system,
            self.x, self.y, self.z, self.r, 0
        )
    
    def press_release_buttons(self, buttons):
        """按钮按下和释放"""
        button_mask = sum(1 << button for button in buttons)
        
        # 按下
        self.master.mav.manual_control_send(
            self.master.target_system,
            self.x, self.y, self.z, self.r, button_mask
        )
        
        # 释放
        self.master.mav.manual_control_send(
            self.master.target_system,
            self.x, self.y, self.z, self.r, 0
        )
    
    def set_target_depth(self, depth):
        """设置目标深度"""
        self.master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.boot_time)),
            self.master.target_system, self.master.target_component,
            coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
            type_mask=(
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            ),
            lat_int=0, lon_int=0, alt=depth,
            vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        )
    
    def set_target_attitude(self, roll, pitch, yaw):
        """设置目标姿态"""
        self.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)),
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
            0, 0, 0, 0
        )
    
    def set_servo_pwm(self, servo_n, microseconds):
        """设置舵机PWM"""
        self.master.set_servo(servo_n + 8, microseconds)
    
    def look_at(self, tilt, roll=0, pan=0):
        """云台控制"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            1, tilt, roll, pan, 0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING
        )
    
    def run(self):
        """运行主循环"""
        rospy.loginfo("ArduSub ROS接口开始运行...")
        
        # 发送心跳
        heartbeat_rate = rospy.Rate(1)  # 1Hz心跳
        
        while not rospy.is_shutdown():
            # 发送心跳
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            
            heartbeat_rate.sleep()


if __name__ == '__main__':
    try:
        import std_msgs.msg  # 确保导入std_msgs.msg
        
        interface = ArduSubROSInterface()
        interface.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"程序异常退出: {e}")
        sys.exit(1)
