#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub ROS接口使用示例
演示如何在实际应用中使用ROS接口控制ArduSub
"""

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import Imu

class SimpleArduSubController:
    """简单的ArduSub控制器示例"""
    
    def __init__(self):
        rospy.init_node('simple_ardusub_controller', anonymous=True)
        
        # 控制发布者
        self.cmd_vel_pub = rospy.Publisher('/ardusub/cmd_vel', Twist, queue_size=10)
        self.arm_pub = rospy.Publisher('/ardusub/arm', Bool, queue_size=10)
        self.mode_pub = rospy.Publisher('/ardusub/set_mode', String, queue_size=10)
        
        # 状态订阅者
        self.depth_sub = rospy.Subscriber('/ardusub/depth', Float32, self.depth_callback)
        self.imu_sub = rospy.Subscriber('/ardusub/imu', Imu, self.imu_callback)
        self.armed_sub = rospy.Subscriber('/ardusub/armed', Bool, self.armed_callback)
        
        # 状态变量
        self.current_depth = 0.0
        self.is_armed = False
        
        rospy.loginfo("简单ArduSub控制器已初始化")
        time.sleep(2)  # 等待连接建立
    
    def depth_callback(self, msg):
        """深度回调"""
        self.current_depth = msg.data
    
    def imu_callback(self, msg):
        """IMU回调"""
        # 可以在这里处理姿态数据
        pass
    
    def armed_callback(self, msg):
        """解锁状态回调"""
        self.is_armed = msg.data
    
    def arm_vehicle(self):
        """解锁载具"""
        rospy.loginfo("正在解锁ArduSub...")
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)
        
        # 等待解锁确认
        timeout = time.time() + 10  # 10秒超时
        while not self.is_armed and time.time() < timeout:
            time.sleep(0.1)
        
        if self.is_armed:
            rospy.loginfo("ArduSub已成功解锁")
            return True
        else:
            rospy.logwarn("ArduSub解锁失败")
            return False
    
    def disarm_vehicle(self):
        """上锁载具"""
        rospy.loginfo("正在上锁ArduSub...")
        arm_msg = Bool()
        arm_msg.data = False
        self.arm_pub.publish(arm_msg)
        
        # 等待上锁确认
        timeout = time.time() + 10
        while self.is_armed and time.time() < timeout:
            time.sleep(0.1)
        
        if not self.is_armed:
            rospy.loginfo("ArduSub已成功上锁")
            return True
        else:
            rospy.logwarn("ArduSub上锁失败")
            return False
    
    def set_mode(self, mode):
        """设置飞行模式"""
        rospy.loginfo(f"切换到{mode}模式...")
        mode_msg = String()
        mode_msg.data = mode.upper()
        self.mode_pub.publish(mode_msg)
        time.sleep(2)  # 等待模式切换
    
    def move_forward(self, speed=0.5, duration=2.0):
        """前进"""
        rospy.loginfo(f"前进 - 速度: {speed}, 持续时间: {duration}秒")
        
        twist = Twist()
        twist.linear.x = speed
        
        end_time = time.time() + duration
        rate = rospy.Rate(10)  # 10Hz
        
        while time.time() < end_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def move_up(self, speed=0.3, duration=2.0):
        """上浮"""
        rospy.loginfo(f"上浮 - 速度: {speed}, 持续时间: {duration}秒")
        
        twist = Twist()
        twist.linear.z = speed
        
        end_time = time.time() + duration
        rate = rospy.Rate(10)
        
        while time.time() < end_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止
        twist.linear.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def move_down(self, speed=0.3, duration=2.0):
        """下潜"""
        rospy.loginfo(f"下潜 - 速度: {speed}, 持续时间: {duration}秒")
        
        twist = Twist()
        twist.linear.z = -speed
        
        end_time = time.time() + duration
        rate = rospy.Rate(10)
        
        while time.time() < end_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止
        twist.linear.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def turn_left(self, speed=0.3, duration=2.0):
        """左转"""
        rospy.loginfo(f"左转 - 速度: {speed}, 持续时间: {duration}秒")
        
        twist = Twist()
        twist.angular.z = speed
        
        end_time = time.time() + duration
        rate = rospy.Rate(10)
        
        while time.time() < end_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # 停止
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def stop(self):
        """停止所有运动"""
        rospy.loginfo("停止运动")
        twist = Twist()  # 所有值默认为0
        self.cmd_vel_pub.publish(twist)
    
    def get_status(self):
        """获取当前状态"""
        return {
            'depth': self.current_depth,
            'armed': self.is_armed
        }
    
    def demo_mission(self):
        """演示任务"""
        rospy.loginfo("开始演示任务...")
        
        try:
            # 1. 设置为手动模式
            self.set_mode("MANUAL")
            
            # 2. 解锁
            if not self.arm_vehicle():
                rospy.logerr("无法解锁，任务中止")
                return False
            
            # 3. 简单的运动序列
            rospy.loginfo("执行运动序列...")
            
            # 前进
            self.move_forward(speed=0.3, duration=3.0)
            time.sleep(1)
            
            # 下潜
            self.move_down(speed=0.2, duration=2.0)
            time.sleep(1)
            
            # 左转
            self.turn_left(speed=0.2, duration=2.0)
            time.sleep(1)
            
            # 上浮回到原深度
            self.move_up(speed=0.2, duration=2.0)
            time.sleep(1)
            
            # 后退回到起始位置
            self.move_forward(speed=-0.3, duration=3.0)
            
            # 4. 停止并上锁
            self.stop()
            time.sleep(2)
            
            self.disarm_vehicle()
            
            rospy.loginfo("演示任务完成")
            return True
            
        except Exception as e:
            rospy.logerr(f"任务执行错误: {e}")
            self.stop()
            self.disarm_vehicle()
            return False
    
    def keyboard_control(self):
        """键盘控制模式（模拟，实际使用ROS话题）"""
        rospy.loginfo("键盘控制模式 - 使用ROS话题发送指令")
        rospy.loginfo("可以使用以下ROS命令进行控制:")
        print()
        print("解锁/上锁:")
        print("  rostopic pub /ardusub/arm std_msgs/Bool \"data: true\"")
        print("  rostopic pub /ardusub/arm std_msgs/Bool \"data: false\"")
        print()
        print("运动控制:")
        print("  # 前进")
        print("  rostopic pub /ardusub/cmd_vel geometry_msgs/Twist \"linear: {x: 0.5, y: 0.0, z: 0.0}\"")
        print("  # 后退")
        print("  rostopic pub /ardusub/cmd_vel geometry_msgs/Twist \"linear: {x: -0.5, y: 0.0, z: 0.0}\"")
        print("  # 左移")
        print("  rostopic pub /ardusub/cmd_vel geometry_msgs/Twist \"linear: {x: 0.0, y: 0.5, z: 0.0}\"")
        print("  # 右移")
        print("  rostopic pub /ardusub/cmd_vel geometry_msgs/Twist \"linear: {x: 0.0, y: -0.5, z: 0.0}\"")
        print("  # 上浮")
        print("  rostopic pub /ardusub/cmd_vel geometry_msgs/Twist \"linear: {x: 0.0, y: 0.0, z: 0.5}\"")
        print("  # 下潜")
        print("  rostopic pub /ardusub/cmd_vel geometry_msgs/Twist \"linear: {x: 0.0, y: 0.0, z: -0.5}\"")
        print("  # 停止")
        print("  rostopic pub /ardusub/cmd_vel geometry_msgs/Twist \"linear: {x: 0.0, y: 0.0, z: 0.0}\"")
        print()
        print("模式切换:")
        print("  rostopic pub /ardusub/set_mode std_msgs/String \"data: 'MANUAL'\"")
        print("  rostopic pub /ardusub/set_mode std_msgs/String \"data: 'STABILIZE'\"")
        print()
        print("查看状态:")
        print("  rostopic echo /ardusub/depth")
        print("  rostopic echo /ardusub/armed")
        print("  rostopic echo /ardusub/imu")
        print()
        
        # 保持运行，监控状态
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            status = self.get_status()
            print(f"\\r深度: {status['depth']:.2f}m, 解锁: {'是' if status['armed'] else '否'}", end='', flush=True)
            rate.sleep()


def main():
    try:
        controller = SimpleArduSubController()
        
        print("ArduSub简单控制器")
        print("1. 运行演示任务")
        print("2. 显示键盘控制说明")
        print("3. 仅监控状态")
        
        choice = input("请选择模式 (1/2/3): ").strip()
        
        if choice == "1":
            controller.demo_mission()
        elif choice == "2":
            controller.keyboard_control()
        elif choice == "3":
            rospy.loginfo("状态监控模式 - 按Ctrl+C退出")
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                status = controller.get_status()
                print(f"\\r深度: {status['depth']:.2f}m, 解锁: {'是' if status['armed'] else '否'}", end='', flush=True)
                rate.sleep()
        else:
            print("无效选择")
            
    except rospy.ROSInterruptException:
        print("\\n程序被用户中断")
    except KeyboardInterrupt:
        print("\\n程序被用户中断")
    except Exception as e:
        print(f"\\n程序异常退出: {e}")


if __name__ == '__main__':
    main()
