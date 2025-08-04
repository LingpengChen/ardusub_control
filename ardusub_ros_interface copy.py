#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub ROS接口增强版
连接ArduSub飞控，提供完整的ROS话题接口
包括控制指令订阅和状态数据发布
"""

import rospy
import sys
import time
import math
import threading
from typing import Dict, Any

# ROS消息类型
from std_msgs.msg import Header, Float32, Int32, Bool, String
from geometry_msgs.msg import Twist, Vector3, Quaternion, PoseStamped
from sensor_msgs.msg import Imu, FluidPressure, Temperature, BatteryState
from nav_msgs.msg import Odometry

# MAVLink相关
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

class ArduSubROSInterfaceAdvanced:
    """ArduSub高级ROS接口类"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('ardusub_ros_interface_advanced', anonymous=True)
        rospy.loginfo("初始化ArduSub高级ROS接口...")
        
        # 连接参数
        self.connection_string = rospy.get_param('~connection_string', 'udpin:0.0.0.0:14551')
        self.data_rate = rospy.get_param('~data_rate', 10)  # Hz
        self.heartbeat_rate = rospy.get_param('~heartbeat_rate', 1)  # Hz
        
        # 状态变量
        self.boot_time = time.time()
        self.connected = False
        self.armed = False
        self.current_mode = "UNKNOWN"
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_remaining = 0.0
        
        # 控制变量
        self.control_input = {
            'x': 0, 'y': 0, 'z': 500, 'r': 0,  # z=500为中性位置
            'buttons': 0
        }
        
        # 传感器数据
        self.sensor_data = {
            'imu': None,
            'pressure': None,
            'temperature': None,
            'position': None,
            'attitude': None
        }
        
        # 初始化MAVLink连接
        self.master = None
        self.init_mavlink_connection()
        
        # 设置ROS发布者和订阅者
        self.setup_publishers()
        self.setup_subscribers()
        
        # 启动数据处理线程
        self.start_threads()
        
        rospy.loginfo("ArduSub高级ROS接口初始化完成")
    
    def init_mavlink_connection(self):
        """初始化MAVLink连接"""
        try:
            rospy.loginfo(f"连接到ArduSub: {self.connection_string}")
            self.master = mavutil.mavlink_connection(self.connection_string)
            
            # 等待心跳
            rospy.loginfo("等待ArduSub心跳...")
            self.master.wait_heartbeat(timeout=10)
            rospy.loginfo("ArduSub连接成功!")
            
            # 请求数据流
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                self.data_rate,
                1
            )
            
            self.connected = True
            
        except Exception as e:
            rospy.logerr(f"MAVLink连接失败: {e}")
            self.connected = False
    
    def setup_publishers(self):
        """设置ROS发布者"""
        # 传感器数据发布者
        self.imu_pub = rospy.Publisher('/ardusub/imu', Imu, queue_size=10)
        self.pressure_pub = rospy.Publisher('/ardusub/pressure', FluidPressure, queue_size=10)
        self.temperature_pub = rospy.Publisher('/ardusub/temperature', Temperature, queue_size=10)
        self.battery_pub = rospy.Publisher('/ardusub/battery', BatteryState, queue_size=10)
        
        # 位置和姿态发布者
        self.odometry_pub = rospy.Publisher('/ardusub/odometry', Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher('/ardusub/pose', PoseStamped, queue_size=10)
        
        # 状态发布者
        self.status_pub = rospy.Publisher('/ardusub/status', String, queue_size=10)
        self.armed_pub = rospy.Publisher('/ardusub/armed_status', Bool, queue_size=10)
        self.mode_pub = rospy.Publisher('/ardusub/current_mode', String, queue_size=10)
        self.connection_pub = rospy.Publisher('/ardusub/connection_status', Bool, queue_size=10)
        
        # 深度发布者
        self.depth_pub = rospy.Publisher('/ardusub/current_depth', Float32, queue_size=10)
    
    def setup_subscribers(self):
        """设置ROS订阅者"""
        # 控制指令订阅者
        rospy.Subscriber('/ardusub/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/ardusub/arm', Bool, self.arm_callback)
        rospy.Subscriber('/ardusub/set_mode', String, self.set_mode_callback)
        rospy.Subscriber('/ardusub/lights', Int32, self.lights_callback)
        rospy.Subscriber('/ardusub/target_depth', Float32, self.target_depth_callback)
        rospy.Subscriber('/ardusub/gimbal_control', Vector3, self.gimbal_callback)
        rospy.Subscriber('/ardusub/servo_control', Vector3, self.servo_callback)
    
    def cmd_vel_callback(self, msg):
        """处理速度控制指令"""
        # 将Twist消息转换为MAVLink手动控制指令
        # 将速度值映射到PWM范围 (1100-1900, 1500为中性)
        self.control_input['x'] = int(msg.linear.x)   # 前后
        self.control_input['y'] = int(msg.linear.y)   # 左右
        self.control_input['z'] = int(msg.linear.z + 500)  # 升降，500为中性
        self.control_input['r'] = int(msg.angular.z)  # 旋转
        
        self.send_manual_control()
    
    def arm_callback(self, msg):
        """处理解锁/上锁指令"""
        if msg.data:
            self.arm_vehicle()
        else:
            self.disarm_vehicle()
    
    def set_mode_callback(self, msg):
        """处理模式切换指令"""
        self.set_flight_mode(msg.data)
    
    def lights_callback(self, msg):
        """处理灯光控制指令"""
        self.set_lights(msg.data)
    
    def target_depth_callback(self, msg):
        """处理目标深度设置"""
        self.set_target_depth(msg.data)
    
    def gimbal_callback(self, msg):
        """处理云台控制指令"""
        self.control_gimbal(msg.y, msg.x, msg.z)  # tilt, roll, pan
    
    def servo_callback(self, msg):
        """处理舵机控制指令"""
        # msg.x, msg.y, msg.z 分别对应不同的舵机
        if msg.x != 0:
            self.set_servo_pwm(9, int(msg.x))  # 舵机1
        if msg.y != 0:
            self.set_servo_pwm(10, int(msg.y)) # 舵机2
        if msg.z != 0:
            self.set_servo_pwm(11, int(msg.z)) # 舵机3
    
    def send_manual_control(self):
        """发送手动控制指令"""
        if not self.connected or not self.master:
            return
        
        try:
            self.master.mav.manual_control_send(
                self.master.target_system,
                self.control_input['x'],  # x (前后)
                self.control_input['y'],  # y (左右)
                self.control_input['z'],  # z (升降)
                self.control_input['r'],  # r (旋转)
                self.control_input['buttons']  # 按钮状态
            )
        except Exception as e:
            rospy.logerr(f"发送手动控制指令失败: {e}")
    
    def arm_vehicle(self):
        """解锁飞行器"""
        if not self.connected or not self.master:
            return
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            rospy.loginfo("发送解锁指令")
        except Exception as e:
            rospy.logerr(f"解锁失败: {e}")
    
    def disarm_vehicle(self):
        """上锁飞行器"""
        if not self.connected or not self.master:
            return
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0
            )
            rospy.loginfo("发送上锁指令")
        except Exception as e:
            rospy.logerr(f"上锁失败: {e}")
    
    def set_flight_mode(self, mode_name):
        """设置飞行模式"""
        if not self.connected or not self.master:
            return
        
        mode_mapping = {
            'MANUAL': 0,
            'STABILIZE': 1,
            'DEPTH_HOLD': 2,
            'POSHOLD': 16,
            'AUTO': 3,
            'GUIDED': 4,
            'CIRCLE': 7,
            'SURFACE': 9,
        }
        
        if mode_name not in mode_mapping:
            rospy.logwarn(f"未知飞行模式: {mode_name}")
            return
        
        try:
            mode_id = mode_mapping[mode_name]
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            rospy.loginfo(f"设置飞行模式: {mode_name}")
        except Exception as e:
            rospy.logerr(f"设置飞行模式失败: {e}")
    
    def set_lights(self, brightness):
        """设置灯光亮度 (0-100)"""
        if not self.connected or not self.master:
            return
        
        try:
            # 将亮度转换为PWM值 (1100-1900)
            pwm_value = int(1100 + (brightness / 100.0) * 800)
            pwm_value = max(1100, min(1900, pwm_value))
            
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                9,  # 灯光通道 (ArduSub lights are on channel 9)
                pwm_value,
                0, 0, 0, 0, 0
            )
            rospy.loginfo(f"设置灯光亮度: {brightness}% (PWM: {pwm_value})")
        except Exception as e:
            rospy.logerr(f"设置灯光失败: {e}")
    
    def set_target_depth(self, depth):
        """设置目标深度"""
        if not self.connected or not self.master:
            return
        
        try:
            self.master.mav.set_position_target_global_int_send(
                int(1e3 * (time.time() - self.boot_time)),
                self.master.target_system,
                self.master.target_component,
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
                vx=0, vy=0, vz=0,
                afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
            )
            rospy.loginfo(f"设置目标深度: {depth}m")
        except Exception as e:
            rospy.logerr(f"设置目标深度失败: {e}")
    
    def control_gimbal(self, tilt, roll=0, pan=0):
        """控制云台"""
        if not self.connected or not self.master:
            return
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                1,
                tilt, roll, pan,
                0, 0, 0,
                mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING
            )
            rospy.loginfo(f"控制云台: 俯仰={tilt}°, 横滚={roll}°, 偏航={pan}°")
        except Exception as e:
            rospy.logerr(f"控制云台失败: {e}")
    
    def set_servo_pwm(self, servo_n, pwm):
        """设置舵机PWM值"""
        if not self.connected or not self.master:
            return
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                servo_n, pwm,
                0, 0, 0, 0, 0
            )
            rospy.loginfo(f"设置舵机{servo_n}: {pwm}")
        except Exception as e:
            rospy.logerr(f"设置舵机失败: {e}")
    
    def process_mavlink_messages(self):
        """处理MAVLink消息"""
        while not rospy.is_shutdown() and self.connected:
            try:
                msg = self.master.recv_match(blocking=False, timeout=0.1)
                if msg is None:
                    continue
                
                msg_type = msg.get_type()
                
                if msg_type == 'HEARTBEAT':
                    self.process_heartbeat(msg)
                elif msg_type == 'ATTITUDE':
                    self.process_attitude(msg)
                elif msg_type == 'SCALED_PRESSURE':
                    self.process_pressure(msg)
                elif msg_type == 'SCALED_IMU2':
                    self.process_imu(msg)
                elif msg_type == 'BATTERY_STATUS':
                    self.process_battery(msg)
                elif msg_type == 'GLOBAL_POSITION_INT':
                    self.process_global_position(msg)
                elif msg_type == 'LOCAL_POSITION_NED':
                    self.process_local_position(msg)
                elif msg_type == 'SYS_STATUS':
                    self.process_system_status(msg)
                    
            except Exception as e:
                rospy.logerr(f"处理MAVLink消息错误: {e}")
                time.sleep(0.1)
    
    def process_heartbeat(self, msg):
        """处理心跳消息"""
        self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        
        # 发布解锁状态
        armed_msg = Bool()
        armed_msg.data = self.armed
        self.armed_pub.publish(armed_msg)
        
        # 发布连接状态
        connection_msg = Bool()
        connection_msg.data = self.connected
        self.connection_pub.publish(connection_msg)
    
    def process_attitude(self, msg):
        """处理姿态消息"""
        # 创建IMU消息
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "base_link"
        
        # 从欧拉角转换为四元数
        quaternion = QuaternionBase([msg.roll, msg.pitch, msg.yaw])
        imu_msg.orientation.x = quaternion.q[1]
        imu_msg.orientation.y = quaternion.q[2]
        imu_msg.orientation.z = quaternion.q[3]
        imu_msg.orientation.w = quaternion.q[0]
        
        # 角速度
        imu_msg.angular_velocity.x = msg.rollspeed
        imu_msg.angular_velocity.y = msg.pitchspeed
        imu_msg.angular_velocity.z = msg.yawspeed
        
        self.imu_pub.publish(imu_msg)
        
        # 创建位姿消息
        pose_msg = PoseStamped()
        pose_msg.header = imu_msg.header
        pose_msg.pose.orientation = imu_msg.orientation
        self.pose_pub.publish(pose_msg)
    
    def process_pressure(self, msg):
        """处理压力消息"""
        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = rospy.Time.now()
        pressure_msg.header.frame_id = "base_link"
        pressure_msg.fluid_pressure = msg.press_abs * 100  # 转换为Pa
        pressure_msg.variance = 0.0
        
        self.pressure_pub.publish(pressure_msg)
        
        # 计算并发布深度
        depth = (msg.press_abs * 100 - 101325) / 9806.65  # 转换为米
        depth_msg = Float32()
        depth_msg.data = depth
        self.depth_pub.publish(depth_msg)
        
        # 发布温度
        temp_msg = Temperature()
        temp_msg.header = pressure_msg.header
        temp_msg.temperature = msg.temperature / 100.0  # 转换为摄氏度
        temp_msg.variance = 0.0
        self.temperature_pub.publish(temp_msg)
    
    def process_imu(self, msg):
        """处理IMU消息"""
        # 这里可以处理更详细的IMU数据
        pass
    
    def process_battery(self, msg):
        """处理电池状态消息"""
        battery_msg = BatteryState()
        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = msg.voltages[0] / 1000.0 if msg.voltages else 0.0
        battery_msg.current = msg.current_battery / 100.0 if msg.current_battery > 0 else 0.0
        battery_msg.charge = float('nan')
        battery_msg.capacity = float('nan')  
        battery_msg.design_capacity = float('nan')
        battery_msg.percentage = msg.battery_remaining / 100.0 if msg.battery_remaining >= 0 else float('nan')
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        battery_msg.present = True
        
        self.battery_pub.publish(battery_msg)
        
        # 更新内部状态
        self.battery_voltage = battery_msg.voltage
        self.battery_current = battery_msg.current
        self.battery_remaining = msg.battery_remaining
    
    def process_global_position(self, msg):
        """处理全局位置消息"""
        # 创建里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # 位置 (注意坐标系转换)
        odom_msg.pose.pose.position.x = msg.lat / 1e7  # 纬度
        odom_msg.pose.pose.position.y = msg.lon / 1e7  # 经度
        odom_msg.pose.pose.position.z = msg.alt / 1000.0  # 高度
        
        # 速度
        odom_msg.twist.twist.linear.x = msg.vx / 100.0
        odom_msg.twist.twist.linear.y = msg.vy / 100.0
        odom_msg.twist.twist.linear.z = msg.vz / 100.0
        
        self.odometry_pub.publish(odom_msg)
    
    def process_local_position(self, msg):
        """处理本地位置消息"""
        # 这里可以处理本地坐标系的位置信息
        pass
    
    def process_system_status(self, msg):
        """处理系统状态消息"""
        # 发布系统状态字符串
        status_str = f"Voltage: {msg.voltage_battery/1000.0:.1f}V, " \
                    f"Current: {msg.current_battery/100.0:.1f}A, " \
                    f"Remaining: {msg.battery_remaining}%"
        
        status_msg = String()
        status_msg.data = status_str
        self.status_pub.publish(status_msg)
    
    def send_heartbeat(self):
        """发送心跳消息"""
        while not rospy.is_shutdown() and self.connected:
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                time.sleep(1.0 / self.heartbeat_rate)
            except Exception as e:
                rospy.logerr(f"发送心跳失败: {e}")
                time.sleep(1.0)
    
    def start_threads(self):
        """启动处理线程"""
        # MAVLink消息处理线程
        mavlink_thread = threading.Thread(target=self.process_mavlink_messages, daemon=True)
        mavlink_thread.start()
        
        # 心跳发送线程
        heartbeat_thread = threading.Thread(target=self.send_heartbeat, daemon=True)
        heartbeat_thread.start()
        
        rospy.loginfo("所有处理线程已启动")
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("ArduSub ROS接口运行中...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("收到退出信号")
        except Exception as e:
            rospy.logerr(f"运行错误: {e}")
        finally:
            if self.master:
                self.master.close()
            rospy.loginfo("ArduSub ROS接口已关闭")

def main():
    try:
        interface = ArduSubROSInterfaceAdvanced()
        interface.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"主程序错误: {e}")

if __name__ == '__main__':
    main()
