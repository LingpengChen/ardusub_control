#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArduSub Status Monitoring UI
A tkinter-based graphical interface to display real-time robot status information
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

# ROS message types
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Bool, String, Int32
from sensor_msgs.msg import Imu, FluidPressure
from nav_msgs.msg import Odometry

class ArduSubStatusUI:
    """ArduSub Status Monitoring UI"""
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ardusub_status_ui', anonymous=True)
        
        # Robot status data
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
        
        # Historical data (for plotting)
        self.history_length = 100
        self.time_history = []
        self.depth_history = []
        self.orientation_history = {'roll': [], 'pitch': [], 'yaw': []}
        
        # Create main window
        self.setup_main_window()
        
        # Create UI components
        self.setup_ui_components()
        
        # Setup ROS subscribers
        self.setup_ros_subscribers()
        
        # Start update thread
        self.start_update_thread()
        
        rospy.loginfo("ArduSub Status Monitoring UI started successfully")
    
    def setup_main_window(self):
        """Setup main window"""
        self.root = tk.Tk()
        self.root.title("ArduSub Status Monitor")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')
        
        # Setup styles
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel', font=('Arial', 14, 'bold'), background='#2b2b2b', foreground='white')
        style.configure('Status.TLabel', font=('Arial', 10), background='#2b2b2b', foreground='white')
        style.configure('Value.TLabel', font=('Arial', 12, 'bold'), background='#2b2b2b', foreground='#00ff00')
        style.configure('Warning.TLabel', font=('Arial', 12, 'bold'), background='#2b2b2b', foreground='#ff4444')
    
    def setup_ui_components(self):
        """Setup UI components"""
        # Main title
        title_frame = tk.Frame(self.root, bg='#2b2b2b')
        title_frame.pack(fill='x', padx=10, pady=5)
        
        title_label = ttk.Label(title_frame, text="ArduSub Underwater Robot Status Monitor", style='Title.TLabel')
        title_label.pack()
        
        # Create main content frame
        main_frame = tk.Frame(self.root, bg='#2b2b2b')
        main_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Left status panel
        self.setup_status_panel(main_frame)
        
        # Right chart panel
        self.setup_chart_panel(main_frame)
        
        # Bottom control panel
        self.setup_control_panel()
    
    def setup_status_panel(self, parent):
        """Setup status information panel"""
        status_frame = tk.Frame(parent, bg='#3b3b3b', relief='raised', bd=2)
        status_frame.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        # System status area
        self.create_status_section(status_frame, "System Status", 0)
        
        # Position and attitude area
        self.create_position_section(status_frame, "Position & Attitude", 1)
        
        # Control status area
        self.create_control_section(status_frame, "Control Status", 2)
        
        # Device status area
        self.create_device_section(status_frame, "Device Status", 3)
    
    def create_status_section(self, parent, title, row):
        """Create system status area"""
        frame = tk.LabelFrame(parent, text=title, bg='#3b3b3b', fg='white', font=('Arial', 12, 'bold'))
        frame.grid(row=row, column=0, sticky='ew', padx=5, pady=5)
        parent.grid_columnconfigure(0, weight=1)
        
        # Motor status
        tk.Label(frame, text="Motor Status:", bg='#3b3b3b', fg='white').grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.armed_label = ttk.Label(frame, text="Locked", style='Warning.TLabel')
        self.armed_label.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        
        # Flight mode
        tk.Label(frame, text="Flight Mode:", bg='#3b3b3b', fg='white').grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.mode_label = ttk.Label(frame, text="UNKNOWN", style='Value.TLabel')
        self.mode_label.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        
        # Connection status
        tk.Label(frame, text="Connection:", bg='#3b3b3b', fg='white').grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.connection_label = ttk.Label(frame, text="Disconnected", style='Warning.TLabel')
        self.connection_label.grid(row=2, column=1, sticky='w', padx=5, pady=2)
    
    def create_position_section(self, parent, title, row):
        """Create position and attitude area"""
        frame = tk.LabelFrame(parent, text=title, bg='#3b3b3b', fg='white', font=('Arial', 12, 'bold'))
        frame.grid(row=row, column=0, sticky='ew', padx=5, pady=5)
        
        # Depth information
        tk.Label(frame, text="Current Depth:", bg='#3b3b3b', fg='white').grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.depth_label = ttk.Label(frame, text="0.0 m", style='Value.TLabel')
        self.depth_label.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="Target Depth:", bg='#3b3b3b', fg='white').grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.target_depth_label = ttk.Label(frame, text="0.0 m", style='Value.TLabel')
        self.target_depth_label.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        
        # Attitude information
        tk.Label(frame, text="Roll Angle:", bg='#3b3b3b', fg='white').grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.roll_label = ttk.Label(frame, text="0.0°", style='Value.TLabel')
        self.roll_label.grid(row=2, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="Pitch Angle:", bg='#3b3b3b', fg='white').grid(row=3, column=0, sticky='w', padx=5, pady=2)
        self.pitch_label = ttk.Label(frame, text="0.0°", style='Value.TLabel')
        self.pitch_label.grid(row=3, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="Yaw Angle:", bg='#3b3b3b', fg='white').grid(row=4, column=0, sticky='w', padx=5, pady=2)
        self.yaw_label = ttk.Label(frame, text="0.0°", style='Value.TLabel')
        self.yaw_label.grid(row=4, column=1, sticky='w', padx=5, pady=2)
    
    def create_control_section(self, parent, title, row):
        """Create control status area"""
        frame = tk.LabelFrame(parent, text=title, bg='#3b3b3b', fg='white', font=('Arial', 12, 'bold'))
        frame.grid(row=row, column=0, sticky='ew', padx=5, pady=5)
        
        # Thruster control
        tk.Label(frame, text="Forward/Backward:", bg='#3b3b3b', fg='white').grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.thrust_x_label = ttk.Label(frame, text="0", style='Value.TLabel')
        self.thrust_x_label.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="Left/Right:", bg='#3b3b3b', fg='white').grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.thrust_y_label = ttk.Label(frame, text="0", style='Value.TLabel')
        self.thrust_y_label.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="Up/Down:", bg='#3b3b3b', fg='white').grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.thrust_z_label = ttk.Label(frame, text="0", style='Value.TLabel')
        self.thrust_z_label.grid(row=2, column=1, sticky='w', padx=5, pady=2)
        
        tk.Label(frame, text="Rotation:", bg='#3b3b3b', fg='white').grid(row=3, column=0, sticky='w', padx=5, pady=2)
        self.thrust_yaw_label = ttk.Label(frame, text="0", style='Value.TLabel')
        self.thrust_yaw_label.grid(row=3, column=1, sticky='w', padx=5, pady=2)
    
    def create_device_section(self, parent, title, row):
        """Create device status area"""
        frame = tk.LabelFrame(parent, text=title, bg='#3b3b3b', fg='white', font=('Arial', 12, 'bold'))
        frame.grid(row=row, column=0, sticky='ew', padx=5, pady=5)
        
        # Light status
        tk.Label(frame, text="Light Brightness:", bg='#3b3b3b', fg='white').grid(row=0, column=0, sticky='w', padx=5, pady=2)
        self.lights_label = ttk.Label(frame, text="0%", style='Value.TLabel')
        self.lights_label.grid(row=0, column=1, sticky='w', padx=5, pady=2)
        
        # Gimbal status
        tk.Label(frame, text="Gimbal Angle:", bg='#3b3b3b', fg='white').grid(row=1, column=0, sticky='w', padx=5, pady=2)
        self.gimbal_label = ttk.Label(frame, text="0°", style='Value.TLabel')
        self.gimbal_label.grid(row=1, column=1, sticky='w', padx=5, pady=2)
        
        # Battery voltage
        tk.Label(frame, text="Battery Voltage:", bg='#3b3b3b', fg='white').grid(row=2, column=0, sticky='w', padx=5, pady=2)
        self.battery_label = ttk.Label(frame, text="12.0V", style='Value.TLabel')
        self.battery_label.grid(row=2, column=1, sticky='w', padx=5, pady=2)
    
    def setup_chart_panel(self, parent):
        """Setup chart panel"""
        chart_frame = tk.Frame(parent, bg='#3b3b3b', relief='raised', bd=2)
        chart_frame.pack(side='right', fill='both', expand=True, padx=(5, 0))
        
        # Create matplotlib charts
        self.fig = Figure(figsize=(8, 6), dpi=100, facecolor='#3b3b3b')
        
        # Depth chart
        self.depth_ax = self.fig.add_subplot(2, 1, 1)
        self.depth_ax.set_title('Depth Changes', color='white')
        self.depth_ax.set_ylabel('Depth (m)', color='white')
        self.depth_ax.tick_params(colors='white')
        self.depth_ax.set_facecolor('#2b2b2b')
        
        # Attitude chart
        self.attitude_ax = self.fig.add_subplot(2, 1, 2)
        self.attitude_ax.set_title('Attitude Changes', color='white')
        self.attitude_ax.set_xlabel('Time (s)', color='white')
        self.attitude_ax.set_ylabel('Angle (°)', color='white')
        self.attitude_ax.tick_params(colors='white')
        self.attitude_ax.set_facecolor('#2b2b2b')
        
        self.fig.tight_layout()
        
        # Embed chart into tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, chart_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)
    
    def setup_control_panel(self):
        """Setup bottom control panel"""
        control_frame = tk.Frame(self.root, bg='#2b2b2b')
        control_frame.pack(fill='x', padx=10, pady=5)
        
        # Refresh button
        refresh_btn = tk.Button(control_frame, text="Refresh Data", command=self.refresh_data,
                               bg='#4CAF50', fg='white', font=('Arial', 10))
        refresh_btn.pack(side='left', padx=5)
        
        # Save log button
        save_btn = tk.Button(control_frame, text="Save Log", command=self.save_log,
                            bg='#2196F3', fg='white', font=('Arial', 10))
        save_btn.pack(side='left', padx=5)
        
        # Status bar
        self.status_bar = tk.Label(control_frame, text="Ready", bg='#2b2b2b', fg='white', 
                                  relief='sunken', anchor='w')
        self.status_bar.pack(side='right', fill='x', expand=True, padx=5)
    
    def setup_ros_subscribers(self):
        """Setup ROS subscribers"""
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
        """Control command callback"""
        self.robot_data['control_input'] = {
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.linear.z,
            'yaw': msg.angular.z
        }
        self.robot_data['connection_status'] = 'Connected'
    
    def imu_callback(self, msg):
        """IMU data callback"""
        # Convert quaternion to Euler angles
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
        """Pressure sensor callback"""
        # Calculate depth (assuming 1bar = 10m water depth)
        self.robot_data['depth'] = (msg.fluid_pressure - 101325) / 9806.65
    
    def odometry_callback(self, msg):
        """Odometry callback"""
        self.robot_data['position'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
    
    def arm_callback(self, msg):
        """Arm status callback"""
        self.robot_data['armed'] = msg.data
    
    def mode_callback(self, msg):
        """Flight mode callback"""
        self.robot_data['mode'] = msg.data
    
    def lights_callback(self, msg):
        """Light control callback"""
        self.robot_data['lights'] = msg.data
    
    def target_depth_callback(self, msg):
        """Target depth callback"""
        self.robot_data['target_depth'] = msg.data
    
    def gimbal_callback(self, msg):
        """Gimbal control callback"""
        self.robot_data['gimbal_tilt'] = msg.y
    
    def update_ui(self):
        """Update UI display"""
        try:
            # Update system status
            armed_text = "Armed" if self.robot_data['armed'] else "Locked"
            armed_style = 'Value.TLabel' if self.robot_data['armed'] else 'Warning.TLabel'
            self.armed_label.configure(text=armed_text, style=armed_style)
            
            self.mode_label.configure(text=self.robot_data['mode'])
            
            conn_text = self.robot_data['connection_status']
            conn_style = 'Value.TLabel' if conn_text == 'Connected' else 'Warning.TLabel'
            self.connection_label.configure(text=conn_text, style=conn_style)
            
            # Update position and attitude
            self.depth_label.configure(text=f"{self.robot_data['depth']:.2f} m")
            self.target_depth_label.configure(text=f"{self.robot_data['target_depth']:.2f} m")
            self.roll_label.configure(text=f"{self.robot_data['orientation']['roll']:.1f}°")
            self.pitch_label.configure(text=f"{self.robot_data['orientation']['pitch']:.1f}°")
            self.yaw_label.configure(text=f"{self.robot_data['orientation']['yaw']:.1f}°")
            
            # Update control status
            self.thrust_x_label.configure(text=f"{self.robot_data['control_input']['x']:.0f}")
            self.thrust_y_label.configure(text=f"{self.robot_data['control_input']['y']:.0f}")
            self.thrust_z_label.configure(text=f"{self.robot_data['control_input']['z']:.0f}")
            self.thrust_yaw_label.configure(text=f"{self.robot_data['control_input']['yaw']:.0f}")
            
            # Update device status
            self.lights_label.configure(text=f"{self.robot_data['lights']}%")
            self.gimbal_label.configure(text=f"{self.robot_data['gimbal_tilt']:.1f}°")
            self.battery_label.configure(text=f"{self.robot_data['battery_voltage']:.1f}V")
            
            # Update charts
            self.update_charts()
            
            # Update status bar
            current_time = time.strftime("%H:%M:%S")
            self.status_bar.configure(text=f"Last updated: {current_time}")
            
        except Exception as e:
            rospy.logerr(f"UI update error: {e}")
    
    def update_charts(self):
        """Update charts"""
        try:
            current_time = time.time()
            
            # Add new data
            self.time_history.append(current_time)
            self.depth_history.append(self.robot_data['depth'])
            self.orientation_history['roll'].append(self.robot_data['orientation']['roll'])
            self.orientation_history['pitch'].append(self.robot_data['orientation']['pitch'])
            self.orientation_history['yaw'].append(self.robot_data['orientation']['yaw'])
            
            # Limit historical data length
            if len(self.time_history) > self.history_length:
                self.time_history.pop(0)
                self.depth_history.pop(0)
                self.orientation_history['roll'].pop(0)
                self.orientation_history['pitch'].pop(0)
                self.orientation_history['yaw'].pop(0)
            
            # If there are enough data points, update charts
            if len(self.time_history) > 1:
                # Convert time to relative time
                relative_time = [(t - self.time_history[0]) for t in self.time_history]
                
                # Update depth chart
                self.depth_ax.clear()
                self.depth_ax.plot(relative_time, self.depth_history, 'cyan', linewidth=2, label='Actual Depth')
                target_depths = [self.robot_data['target_depth']] * len(relative_time)
                self.depth_ax.plot(relative_time, target_depths, 'orange', linewidth=2, linestyle='--', label='Target Depth')
                self.depth_ax.set_title('Depth Changes', color='white')
                self.depth_ax.set_ylabel('Depth (m)', color='white')
                self.depth_ax.tick_params(colors='white')
                self.depth_ax.set_facecolor('#2b2b2b')
                self.depth_ax.legend()
                self.depth_ax.grid(True, alpha=0.3)
                
                # Update attitude chart
                self.attitude_ax.clear()
                self.attitude_ax.plot(relative_time, self.orientation_history['roll'], 'red', linewidth=2, label='Roll')
                self.attitude_ax.plot(relative_time, self.orientation_history['pitch'], 'green', linewidth=2, label='Pitch')
                self.attitude_ax.plot(relative_time, self.orientation_history['yaw'], 'blue', linewidth=2, label='Yaw')
                self.attitude_ax.set_title('Attitude Changes', color='white')
                self.attitude_ax.set_xlabel('Time (s)', color='white')
                self.attitude_ax.set_ylabel('Angle (°)', color='white')
                self.attitude_ax.tick_params(colors='white')
                self.attitude_ax.set_facecolor('#2b2b2b')
                self.attitude_ax.legend()
                self.attitude_ax.grid(True, alpha=0.3)
                
                self.canvas.draw()
                
        except Exception as e:
            rospy.logerr(f"Chart update error: {e}")
    
    def start_update_thread(self):
        """Start update thread"""
        def update_loop():
            while not rospy.is_shutdown():
                try:
                    self.root.after(0, self.update_ui)
                    time.sleep(0.1)  # 10Hz update frequency
                except Exception as e:
                    rospy.logerr(f"Update thread error: {e}")
        
        update_thread = threading.Thread(target=update_loop, daemon=True)
        update_thread.start()
    
    def refresh_data(self):
        """Refresh data"""
        rospy.loginfo("Manual data refresh")
        self.update_ui()
    
    def save_log(self):
        """Save log"""
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"ardusub_log_{timestamp}.txt"
            
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(f"ArduSub Status Log - {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("=" * 50 + "\n")
                f.write(f"Motor Status: {'Armed' if self.robot_data['armed'] else 'Locked'}\n")
                f.write(f"Flight Mode: {self.robot_data['mode']}\n")
                f.write(f"Current Depth: {self.robot_data['depth']:.2f} m\n")
                f.write(f"Target Depth: {self.robot_data['target_depth']:.2f} m\n")
                f.write(f"Roll Angle: {self.robot_data['orientation']['roll']:.1f}°\n")
                f.write(f"Pitch Angle: {self.robot_data['orientation']['pitch']:.1f}°\n")
                f.write(f"Yaw Angle: {self.robot_data['orientation']['yaw']:.1f}°\n")
                f.write(f"Light Brightness: {self.robot_data['lights']}%\n")
                f.write(f"Gimbal Angle: {self.robot_data['gimbal_tilt']:.1f}°\n")
            
            messagebox.showinfo("Save Successful", f"Log saved to: {filename}")
            rospy.loginfo(f"Log saved to: {filename}")
            
        except Exception as e:
            messagebox.showerror("Save Failed", f"Unable to save log: {e}")
            rospy.logerr(f"Failed to save log: {e}")
    
    def run(self):
        """Run UI"""
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
        except Exception as e:
            rospy.logerr(f"UI runtime error: {e}")
    
    def on_closing(self):
        """Handle window closing"""
        rospy.loginfo("Closing ArduSub Status Monitoring UI")
        self.root.quit()
        self.root.destroy()

def main():
    try:
        ui = ArduSubStatusUI()
        ui.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"UI program error: {e}")

if __name__ == '__main__':
    main()
