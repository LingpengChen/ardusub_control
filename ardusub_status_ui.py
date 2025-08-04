#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub 状态监控UI
基于tkinter的图形界面，显示机器人的实时状态信息
"""

import rospy
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

# ROS消息类型
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import Imu, FluidPressure
from nav_msgs.msg import Odometry

class ArduSubStatusUI:
    """ArduSub状态监控UI"""
    
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('ardusub_status_ui', anonymous=True)
        
        # 机器人状态数据
        self.robot_data = {
            'armed': False,
            'mode': 'UNKNOWN',
            'depth': 0.0,
            'target_depth': 0.0,
            'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0},
            'position': {'x': 0, 'y': 0, 'z': 0},
            'velocity': {'linear': {'x': 0, 'y': 0, 'z': 0}, 'angular': {'x': 0, 'y': 0, 'z': 0}},
            'control_input': {'x': 0, 'y': 0, 'z': 0, 'yaw': 0},
            'lights': 0,
            'gimbal_tilt': 0,
            'battery_voltage': 12.0,
            'connection_status': 'Disconnected'
        }
        
        # 历史数据（用于绘图）
        self.history_length = 100
        self.time_history = []
        self.depth_history = []
        self.orientation_history = {'roll': [], 'pitch': [], 'yaw': []}
        
        # 创建主窗口
        self.setup_main_window()
        
        # 创建UI组件
        self.setup_ui_components()
        
        # 设置ROS订阅者
        self.setup_ros_subscribers()
        
        # 启动数据更新线程
        self.start_update_thread()
        
        rospy.loginfo("ArduSub状态监控UI启动完成")
    
    def setup_main_window(self):
        """设置主窗口"""
        self.root = tk.Tk()
        self.root.title("ArduSub 状态监控")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')
        
        # 设置样式
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel', font=('Arial', 14, 'bold'), background='#2b2b2b', foreground='white')
        style.configure('Status.TLabel', font=('Arial', 10), background='#2b2b2b', foreground='white')
        style.configure('Value.TLabel', font=('Arial', 12, 'bold'), background='#2b2b2b', foreground='#00ff00')
        style.configure('Warning.TLabel', font=('Arial', 12, 'bold'), background='#2b2b2b', foreground='#ff4444')
    
    def setup_ui_components(self):
        """设置UI组件"""
        # 主标题
        title_frame = tk.Frame(self.root, bg='#2b2b2b')
        title_frame.pack(fill='x', padx=10, pady=5)
        
        title_label = ttk.Label(title_frame, text="ArduSub 水下机器人状态监控", style='Title.TLabel')
        title_label.pack()
        
        # 创建主要内容框架
        main_frame = tk.Frame(self.root, bg='#2b2b2b')
        main_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # 左侧状态信息面板
        self.setup_status_panel(main_frame)
        
        # 右侧图表面板
        self.setup_chart_panel(main_frame)
        
        # 底部控制面板
        self.setup_control_panel()
    
    def setup_status_panel(self, parent):
        """设置状态信息面板"""
        status_frame = tk.Frame(parent, bg='#3b3b3b', relief='raised', bd=2)
        status_frame.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        # 系统状态区域
        self.create_status_section(status_frame, "系统状态", 0)
        
        # 位置姿态区域
        self.create_position_section(status_frame, "位置与姿态", 1)
        
        # 控制状态区域
        self.create_control_section(status_frame, "控制状态", 2)
        
        # 设备状态区域
        self.create_device_section(status_frame, "设备状态", 3)
    
    def create_status_section(self, parent, title, row):
        """创建系统状态区域"""
        frame = tk.LabelFrame(parent, text=title, bg='#3b3b3b', fg='white', font=('Arial', 12, 'bold'))
        frame.grid(row=row, column=0, sticky='ew', padx=5, pady=5)
        parent.grid_columnconfigure(0, weight=1)
        
        # 解锁状态
        tk.Label(frame, text="电机状态:", bg='#3b3b3b', fg='white').grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.armed_label = ttk.Label(frame, text="上锁", style='Warning.TLabel')
        self.armed_label.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        
        # 飞行模式
        tk.Label(frame, text="飞行模式:", bg='#3b3b3b', fg='white').grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.mode_label = ttk.Label(frame, text="UNKNOWN", style='Value.TLabel')
        self.mode_label.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        
        # 连接状态
        tk.Label(frame, text="连接状态:", bg='#3b3b3b', fg='white').grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.connection_label = ttk.Label(frame, text="未连接", style='Warning.TLabel')
        self.connection_label.grid(row=2, column=1, sticky='w', padx=5, pady=2)
    
    def create_position_section(self, parent, title, row):
        """创建位置姿态区域"""
        frame = tk.LabelFrame(parent, text=title, bg='#3b3b3b', fg='white', font=('Arial', 12, 'bold'))
        frame.grid(row=row, column=0, sticky='ew', padx=5, pady=5)
        
        # 深度信息
        tk.Label(frame, text="当前深度:", bg='#3b3b3b', fg='white').grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.depth_label = ttk.Label(frame, text="0.0 m", style='Value.TLabel')
        self.depth_label.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="目标深度:", bg='#3b3b3b', fg='white').grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.target_depth_label = ttk.Label(frame, text="0.0 m", style='Value.TLabel')
        self.target_depth_label.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        
        # 姿态信息
        tk.Label(frame, text="横滚角(Roll):", bg='#3b3b3b', fg='white').grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.roll_label = ttk.Label(frame, text="0.0°", style='Value.TLabel')
        self.roll_label.grid(row=2, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="俯仰角(Pitch):", bg='#3b3b3b', fg='white').grid(row=3, column=0, sticky='w', padx=5, pady=2)
        self.pitch_label = ttk.Label(frame, text="0.0°", style='Value.TLabel')
        self.pitch_label.grid(row=3, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="偏航角(Yaw):", bg='#3b3b3b', fg='white').grid(row=4, column=0, sticky='w', padx=5, pady=2)
        self.yaw_label = ttk.Label(frame, text="0.0°", style='Value.TLabel')
        self.yaw_label.grid(row=4, column=1, sticky='w', padx=5, pady=2)
    
    def create_control_section(self, parent, title, row):
        """创建控制状态区域"""
        frame = tk.LabelFrame(parent, text=title, bg='#3b3b3b', fg='white', font=('Arial', 12, 'bold'))
        frame.grid(row=row, column=0, sticky='ew', padx=5, pady=5)
        
        # 推进器控制
        tk.Label(frame, text="前后推力:", bg='#3b3b3b', fg='white').grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.thrust_x_label = ttk.Label(frame, text="0", style='Value.TLabel')
        self.thrust_x_label.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="左右推力:", bg='#3b3b3b', fg='white').grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.thrust_y_label = ttk.Label(frame, text="0", style='Value.TLabel')
        self.thrust_y_label.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="升降推力:", bg='#3b3b3b', fg='white').grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.thrust_z_label = ttk.Label(frame, text="0", style='Value.TLabel')
        self.thrust_z_label.grid(row=2, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="旋转推力:", bg='#3b3b3b', fg='white').grid(row=3, column=0, sticky='w', padx=5, pady=2)
        self.thrust_yaw_label = ttk.Label(frame, text="0", style='Value.TLabel')
        self.thrust_yaw_label.grid(row=3, column=1, sticky='w', padx=5, pady=2)
    
    def create_device_section(self, parent, title, row):
        """创建设备状态区域"""
        frame = tk.LabelFrame(parent, text=title, bg='#3b3b3b', fg='white', font=('Arial', 12, 'bold'))
        frame.grid(row=row, column=0, sticky='ew', padx=5, pady=5)
        
        # 灯光状态
        tk.Label(frame, text="灯光亮度:", bg='#3b3b3b', fg='white').grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.lights_label = ttk.Label(frame, text="0%", style='Value.TLabel')
        self.lights_label.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        
        # 云台状态
        tk.Label(frame, text="云台角度:", bg='#3b3b3b', fg='white').grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.gimbal_label = ttk.Label(frame, text="0°", style='Value.TLabel')
        self.gimbal_label.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        
        # 电池电压
        tk.Label(frame, text="电池电压:", bg='#3b3b3b', fg='white').grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.battery_label = ttk.Label(frame, text="12.0V", style='Value.TLabel')
        self.battery_label.grid(row=2, column=1, sticky='w', padx=5, pady=2)
    
    def setup_chart_panel(self, parent):
        """设置图表面板"""
        chart_frame = tk.Frame(parent, bg='#3b3b3b', relief='raised', bd=2)
        chart_frame.pack(side='right', fill='both', expand=True, padx=(5, 0))
        
        # 创建matplotlib图表
        self.fig = Figure(figsize=(8, 6), dpi=100, facecolor='#3b3b3b')
        
        # 深度图表
        self.depth_ax = self.fig.add_subplot(2, 1, 1)
        self.depth_ax.set_title('深度变化', color='white')
        self.depth_ax.set_ylabel('深度 (m)', color='white')
        self.depth_ax.tick_params(colors='white')
        self.depth_ax.set_facecolor('#2b2b2b')
        
        # 姿态图表
        self.attitude_ax = self.fig.add_subplot(2, 1, 2)
        self.attitude_ax.set_title('姿态变化', color='white')
        self.attitude_ax.set_xlabel('时间 (s)', color='white')
        self.attitude_ax.set_ylabel('角度 (°)', color='white')
        self.attitude_ax.tick_params(colors='white')
        self.attitude_ax.set_facecolor('#2b2b2b')
        
        self.fig.tight_layout()
        
        # 嵌入图表到tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, chart_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
    
    def setup_control_panel(self):
        """设置底部控制面板"""
        control_frame = tk.Frame(self.root, bg='#2b2b2b')
        control_frame.pack(fill='x', padx=10, pady=5)
        
        # 刷新按钮
        refresh_btn = tk.Button(control_frame, text="刷新数据", command=self.refresh_data,
                               bg='#4CAF50', fg='white', font=('Arial', 10))
        refresh_btn.pack(side='left', padx=5)
        
        # 保存日志按钮
        save_btn = tk.Button(control_frame, text="保存日志", command=self.save_log,
                            bg='#2196F3', fg='white', font=('Arial', 10))
        save_btn.pack(side='left', padx=5)
        
        # 状态栏
        self.status_bar = tk.Label(control_frame, text="就绪", bg='#2b2b2b', fg='white', 
                                  relief='sunken', anchor='w')
        self.status_bar.pack(side='right', fill='x', expand=True, padx=5)
    
    def setup_ros_subscribers(self):
        """设置ROS订阅者"""
        rospy.Subscriber('/ardusub/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/ardusub/imu', Imu, self.imu_callback)
        rospy.Subscriber('/ardusub/pressure', FluidPressure, self.pressure_callback)
        rospy.Subscriber('/ardusub/odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('/ardusub/arm', Bool, self.arm_callback)
        rospy.Subscriber('/ardusub/set_mode', String, self.mode_callback)
        rospy.Subscriber('/ardusub/lights', Int32, self.lights_callback)
        rospy.Subscriber('/ardusub/target_depth', Float32, self.target_depth_callback)
        rospy.Subscriber('/ardusub/gimbal_control', Vector3, self.gimbal_callback)
    
    def cmd_vel_callback(self, msg):
        """控制指令回调"""
        self.robot_data['control_input'] = {
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.linear.z,
            'yaw': msg.angular.z
        }
        self.robot_data['connection_status'] = 'Connected'
    
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
        self.robot_data['orientation'] = {
            'roll': euler[0] * 180.0 / math.pi,
            'pitch': euler[1] * 180.0 / math.pi,
            'yaw': euler[2] * 180.0 / math.pi
        }
    
    def pressure_callback(self, msg):
        """压力传感器回调"""
        # 计算深度 (假设1bar = 10m水深)
        self.robot_data['depth'] = (msg.fluid_pressure - 101325) / 9806.65
    
    def odometry_callback(self, msg):
        """里程计回调"""
        self.robot_data['position'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
    
    def arm_callback(self, msg):
        """解锁状态回调"""
        self.robot_data['armed'] = msg.data
    
    def mode_callback(self, msg):
        """飞行模式回调"""
        self.robot_data['mode'] = msg.data
    
    def lights_callback(self, msg):
        """灯光控制回调"""
        self.robot_data['lights'] = msg.data
    
    def target_depth_callback(self, msg):
        """目标深度回调"""
        self.robot_data['target_depth'] = msg.data
    
    def gimbal_callback(self, msg):
        """云台控制回调"""
        self.robot_data['gimbal_tilt'] = msg.y
    
    def update_ui(self):
        """更新UI显示"""
        try:
            # 更新系统状态
            armed_text = "解锁" if self.robot_data['armed'] else "上锁"
            armed_style = 'Value.TLabel' if self.robot_data['armed'] else 'Warning.TLabel'
            self.armed_label.configure(text=armed_text, style=armed_style)
            
            self.mode_label.configure(text=self.robot_data['mode'])
            
            conn_text = self.robot_data['connection_status']
            conn_style = 'Value.TLabel' if conn_text == 'Connected' else 'Warning.TLabel'
            self.connection_label.configure(text=conn_text, style=conn_style)
            
            # 更新位置姿态
            self.depth_label.configure(text=f"{self.robot_data['depth']:.2f} m")
            self.target_depth_label.configure(text=f"{self.robot_data['target_depth']:.2f} m")
            self.roll_label.configure(text=f"{self.robot_data['orientation']['roll']:.1f}°")
            self.pitch_label.configure(text=f"{self.robot_data['orientation']['pitch']:.1f}°")
            self.yaw_label.configure(text=f"{self.robot_data['orientation']['yaw']:.1f}°")
            
            # 更新控制状态
            self.thrust_x_label.configure(text=f"{self.robot_data['control_input']['x']:.0f}")
            self.thrust_y_label.configure(text=f"{self.robot_data['control_input']['y']:.0f}")
            self.thrust_z_label.configure(text=f"{self.robot_data['control_input']['z']:.0f}")
            self.thrust_yaw_label.configure(text=f"{self.robot_data['control_input']['yaw']:.0f}")
            
            # 更新设备状态
            self.lights_label.configure(text=f"{self.robot_data['lights']}%")
            self.gimbal_label.configure(text=f"{self.robot_data['gimbal_tilt']:.1f}°")
            self.battery_label.configure(text=f"{self.robot_data['battery_voltage']:.1f}V")
            
            # 更新图表
            self.update_charts()
            
            # 更新状态栏
            current_time = time.strftime("%H:%M:%S")
            self.status_bar.configure(text=f"最后更新: {current_time}")
            
        except Exception as e:
            rospy.logerr(f"UI更新错误: {e}")
    
    def update_charts(self):
        """更新图表"""
        try:
            current_time = time.time()
            
            # 添加新数据
            self.time_history.append(current_time)
            self.depth_history.append(self.robot_data['depth'])
            self.orientation_history['roll'].append(self.robot_data['orientation']['roll'])
            self.orientation_history['pitch'].append(self.robot_data['orientation']['pitch'])
            self.orientation_history['yaw'].append(self.robot_data['orientation']['yaw'])
            
            # 限制历史数据长度
            if len(self.time_history) > self.history_length:
                self.time_history.pop(0)
                self.depth_history.pop(0)
                self.orientation_history['roll'].pop(0)
                self.orientation_history['pitch'].pop(0)
                self.orientation_history['yaw'].pop(0)
            
            # 如果有足够的数据点，更新图表
            if len(self.time_history) > 1:
                # 转换时间为相对时间
                relative_time = [(t - self.time_history[0]) for t in self.time_history]
                
                # 更新深度图表
                self.depth_ax.clear()
                self.depth_ax.plot(relative_time, self.depth_history, 'cyan', linewidth=2, label='实际深度')
                target_depths = [self.robot_data['target_depth']] * len(relative_time)
                self.depth_ax.plot(relative_time, target_depths, 'orange', linewidth=2, linestyle='--', label='目标深度')
                self.depth_ax.set_title('深度变化', color='white')
                self.depth_ax.set_ylabel('深度 (m)', color='white')
                self.depth_ax.tick_params(colors='white')
                self.depth_ax.set_facecolor('#2b2b2b')
                self.depth_ax.legend()
                self.depth_ax.grid(True, alpha=0.3)
                
                # 更新姿态图表
                self.attitude_ax.clear()
                self.attitude_ax.plot(relative_time, self.orientation_history['roll'], 'red', linewidth=2, label='Roll')
                self.attitude_ax.plot(relative_time, self.orientation_history['pitch'], 'green', linewidth=2, label='Pitch')
                self.attitude_ax.plot(relative_time, self.orientation_history['yaw'], 'blue', linewidth=2, label='Yaw')
                self.attitude_ax.set_title('姿态变化', color='white')
                self.attitude_ax.set_xlabel('时间 (s)', color='white')
                self.attitude_ax.set_ylabel('角度 (°)', color='white')
                self.attitude_ax.tick_params(colors='white')
                self.attitude_ax.set_facecolor('#2b2b2b')
                self.attitude_ax.legend()
                self.attitude_ax.grid(True, alpha=0.3)
                
                self.canvas.draw()
                
        except Exception as e:
            rospy.logerr(f"图表更新错误: {e}")
    
    def start_update_thread(self):
        """启动更新线程"""
        def update_loop():
            while not rospy.is_shutdown():
                try:
                    self.root.after(0, self.update_ui)
                    time.sleep(0.1)  # 10Hz更新频率
                except Exception as e:
                    rospy.logerr(f"更新线程错误: {e}")
        
        update_thread = threading.Thread(target=update_loop, daemon=True)
        update_thread.start()
    
    def refresh_data(self):
        """刷新数据"""
        rospy.loginfo("手动刷新数据")
        self.update_ui()
    
    def save_log(self):
        """保存日志"""
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"ardusub_log_{timestamp}.txt"
            
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(f"ArduSub 状态日志 - {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("=" * 50 + "\n")
                f.write(f"电机状态: {'解锁' if self.robot_data['armed'] else '上锁'}\n")
                f.write(f"飞行模式: {self.robot_data['mode']}\n")
                f.write(f"当前深度: {self.robot_data['depth']:.2f} m\n")
                f.write(f"目标深度: {self.robot_data['target_depth']:.2f} m\n")
                f.write(f"横滚角: {self.robot_data['orientation']['roll']:.1f}°\n")
                f.write(f"俯仰角: {self.robot_data['orientation']['pitch']:.1f}°\n")
                f.write(f"偏航角: {self.robot_data['orientation']['yaw']:.1f}°\n")
                f.write(f"灯光亮度: {self.robot_data['lights']}%\n")
                f.write(f"云台角度: {self.robot_data['gimbal_tilt']:.1f}°\n")
            
            messagebox.showinfo("保存成功", f"日志已保存到: {filename}")
            rospy.loginfo(f"日志已保存到: {filename}")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"无法保存日志: {e}")
            rospy.logerr(f"保存日志失败: {e}")
    
    def run(self):
        """运行UI"""
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
        except Exception as e:
            rospy.logerr(f"UI运行错误: {e}")
    
    def on_closing(self):
        """关闭窗口时的处理"""
        rospy.loginfo("关闭ArduSub状态监控UI")
        self.root.quit()
        self.root.destroy()

def main():
    try:
        ui = ArduSubStatusUI()
        ui.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"UI程序错误: {e}")

if __name__ == '__main__':
    main()
