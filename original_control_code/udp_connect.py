# -*- coding: utf-8 -*-
import socket
import threading
import control_ardusub
import os
import time
from pymavlink import mavutil
import struct
import math


ardusub_connect = control_ardusub.ardusub_control()

class UDPProxy(object):
    

    roll_angle = pitch_angle = yaw_angle = 0

    def __init__(self, local_host, local_port,server_host,server_port):

        print("[UDP] init")

        self.local_address = (local_host, local_port)
        self.server_address = (server_host, server_port)


        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  



        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.tempture = 0.0
        self.depth = 0.0

        print("[UDP] 启动")

        self.recv_thread = threading.Thread(target=self.recv)  
        self.recv_thread.daemon = True 
        self.recv_thread.start() 

        print("[UDP]接收 启动")

        self.send_thread = threading.Thread(target=self.send)  
        self.send_thread.daemon = True 
        self.send_thread.start() 

        print("[UDP]发送 启动")

        self.x=0
        self.y=0
        self.z=500
        self.r=0
        self.set_pwm =0
        self.status =0




    def recv(self):
        self.sock.bind(self.local_address)
        control_ardusub.master.arducopter_arm()
        control_ardusub.master.motors_armed_wait()
        #print("Armed")
        while True:
            # print("等待数据")
            data, client_address = self.sock.recvfrom(1024)
            #print("[From {}]: {}".format(client_address, data))
            data = data.decode('latin1')  
            self.data_prase(data)


    # z:-500 ~ 1500 中间值为500  PWM的范围是 5.5%-9.5%
    # y: -2000 ~ 2000 中间值为0
    # x: -2000 ~ 2000 中间值为0
    # r: -2000 ~ 2000 中间值为0

    def data_prase(self, data):

        if(ord(data[4]) == 0x0b and ord(data[5])==0xfb):
            print("使能控制")
            if(ord(data[0]) != 0xff or ord(data[1]) != 0xfe or ord(data[2]) != 0xfd or ord(data[3]) != 0xfc):
                print("数据包错误")
                return
            if(ord(data[6])==0x00):
                control_ardusub.master.arducopter_disarm()
            elif(ord(data[6])==0x01):
                control_ardusub.master.arducopter_arm()

        elif(ord(data[4]) == 0x0b and ord(data[5])!=0xfb):
            print("运动控制")
            if(ord(data[0]) != 0xff or ord(data[1]) != 0xfe or ord(data[2]) != 0xfd or ord(data[3]) != 0xfc):
                print("数据包错误")
                return
            self.set_pwm = int(ord(data[6]))
            if(self.set_pwm < 127):
                print("反向运行")
                self.set_pwm = self.set_pwm - 127
                self.set_pwm = self.set_pwm * 3.5
                self.set_pwm = int(self.set_pwm)
            elif(self.set_pwm > 128):
                print("正向运行")
                self.set_pwm = self.set_pwm - 128
                self.set_pwm = self.set_pwm * 3.5
                self.set_pwm = int(self.set_pwm)
            else:
                print("停止")
                self.set_pwm = 0

            self.set_pwm = self.set_pwm + 1500

            print(self.set_pwm)  # 根据现象来看，怀疑对应的范围已经发生了改变，不再是1100-1900了

            if(ord(data[5]) == 0x01):
                print("上浮和下潜")
                ardusub_connect.set_rc_channel_pwm(3, self.set_pwm)
                

            elif(ord(data[5]) == 0x02):
                print("左平移和右平移")
                ardusub_connect.set_rc_channel_pwm(6, self.set_pwm)

            elif(ord(data[5]) == 0x03):
                print("前进和后退")
                ardusub_connect.set_rc_channel_pwm(5, self.set_pwm)


            elif(ord(data[5]) == 0x04):
                print("航向控制")
                ardusub_connect.set_rc_channel_pwm(4, self.set_pwm)

            elif(ord(data[5]) == 0x05):
                print("俯仰控制")
                ardusub_connect.set_rc_channel_pwm(1, self.set_pwm)

            elif(ord(data[5]) == 0x06):
                print("横滚控制")
                ardusub_connect.set_rc_channel_pwm(2, self.set_pwm)
            

            elif(ord(data[5]) == 0x0A):
                set_misson = int(ord(data[6]))
                print("机械手张合")
                if(set_misson == 1):
                    print("机械手合")
                    control_ardusub.press_release_buttons(control_ardusub.master,[3],self.x,self.y,self.z,self.r)
                elif(set_misson == 2):
                    print("机械手张")
                    control_ardusub.press_release_buttons(control_ardusub.master,[0],self.x,self.y,self.z,self.r)
                
                
            elif(ord(data[5]) == 0x0B):
                set_misson = int(ord(data[6]))
                ardusub_connect.set_servo_pwm(1,self.set_pwm)
                print("机械手转动")
                

            elif(ord(data[5]) == 0x0C):
                set_misson = int(ord(data[6]))
                if(set_misson == 1):
                    control_ardusub.press_release_buttons(control_ardusub.master,[13],self.x,self.y,self.z,self.r)
                    print("主灯灭")
                elif(set_misson == 2):
                    control_ardusub.press_release_buttons(control_ardusub.master,[14],self.x,self.y,self.z,self.r)
                    print("主灯亮")
                

            elif(ord(data[5]) == 0x0D):
                set_misson = int(ord(data[6]))
                if(set_misson == 1):
                    control_ardusub.press_release_buttons(control_ardusub.master,[9],self.x,self.y,self.z,self.r)
                    print("光圈灭")
                elif(set_misson == 2):
                    control_ardusub.press_release_buttons(control_ardusub.master,[10],self.x,self.y,self.z,self.r)
                    print("光圈亮")
                

            elif(ord(data[5]) == 0x0E):
                set_misson = int(ord(data[6]))
                ardusub_connect.look_at((set_misson-40)*100)
                print(set_misson)
                print("相机云台的倾斜角度")
                

        elif(ord(data[4]) == 0x0e):
            print("灵活运动")
            if(ord(data[0]) != 0xff or ord(data[1]) != 0xfe or ord(data[2]) != 0xfd or ord(data[3]) != 0xfc or ord(data[5]) != 0x14):
                print("数据包错误")
                return
            
            self.x = int(ord(data[6]))
            if(self.x < 127):
                print("反向运行")
                self.x = self.x - 127
                self.x = self.x * 3.5
                self.x = int(self.x)
            elif(self.x > 128):
                print("正向运行")
                self.x = self.x - 128
                self.x = self.x * 3.5
                self.x = int(self.x)
            else:
                print("停止")
                self.x = 0
            self.x = self.x * 4.5

            self.y = int(ord(data[7]))
            if(self.y < 127):
                print("反向运行")
                self.y = self.y - 127
                self.y = self.y * 3.5
                self.y = int(self.y)
            elif(self.y > 128):
                print("正向运行")
                self.y = self.y - 128
                self.y = self.y * 3.5
                self.y = int(self.y)
            else:
                print("停止")
                self.y = 0
            self.y = self.y * 4.5

            self.z = int(ord(data[8]))
            if(self.z < 127):
                print("反向运行")
                self.z = self.z - 127
                self.z = self.z * 3.5
                self.z = int(self.z)
            elif(self.z > 128):
                print("正向运行")
                self.z = self.z - 128
                self.z = self.z * 3.5
                self.z = int(self.z)
            else:
                print("停止")
                self.z = 0
            self.z = self.z * 2.25+500
            
            self.r = int(ord(data[9]))
            if(self.r < 127):
                print("反向运行")
                self.r = self.r - 127
                self.r = self.r * 3.5
                self.r = int(self.r)
            elif(self.r > 128):
                print("正向运行")
                self.r = self.r - 128
                self.r = self.r * 3.5
                self.r = int(self.r)
            else:
                print("停止")
                self.r = 0
            self.r = self.r * 4.5

            self.x = int(self.x)
            self.y = int(self.y)
            self.z = int(self.z)
            self.r = int(self.r)

            control_ardusub.press_release_buttons(control_ardusub.master,0,self.x,self.y,self.z,self.r)
            
             
        elif(ord(data[4])==0x0a and ord(data[5])==0x00):
            if(ord(data[0]) != 0xff or ord(data[1]) != 0xfe or ord(data[2]) != 0xfd or ord(data[3]) != 0xfc):
                print("数据包错误")
                return
            print("heart beat")
            control_ardusub.heartbeat_trigetr()

        elif(ord(data[4]) == 0x0a and ord(data[5]) != 0x00):
            print("模式控制")
            if(ord(data[0]) != 0xff or ord(data[1]) != 0xfe or ord(data[2]) != 0xfd or ord(data[3]) != 0xfc):
                print("数据包错误")
                return
            
            if(ord(data[5]) == 0x07):
                control_ardusub.press_release_buttons(control_ardusub.master,[0,5],self.x,self.y,self.z,self.r)
                print("切换到手动模式")
                
            elif(ord(data[5]) == 0x08):
                control_ardusub.press_release_buttons(control_ardusub.master,[1,5],self.x,self.y,self.z,self.r)
                print("切换到自稳模式")

            elif(ord(data[5]) == 0x09):
                control_ardusub.press_release_buttons(control_ardusub.master,[3,5],self.x,self.y,self.z,self.r)
                print("切换到定深模式")

        elif(ord(data[4]) == 0x0c and ord(data[5])==0x10):
            print("深度设定")
            if(ord(data[0]) != 0xff or ord(data[1]) != 0xfe or ord(data[2]) != 0xfd or ord(data[3]) != 0xfc):
                print("数据包错误")
                return
            control_ardusub.press_release_buttons(control_ardusub.master,[3,5],self.x,self.y,self.z,self.r)
            set_depth = int(ord(data[6]))*256+int(ord(data[7]))
            print(set_depth)
            ardusub_connect.set_target_depth((set_depth/100)*(-1))

        elif(ord(data[4]) == 0x0c):
            
            print("角度设定")
            if(ord(data[0]) != 0xff or ord(data[1]) != 0xfe or ord(data[2]) != 0xfd or ord(data[3]) != 0xfc):
                print("数据包错误")
                return
    
            set_angle = int(ord(data[6]))*256+int(ord(data[7]))
            if(ord(data[5]) == 0x11):
                if(ord(data[5]) == 0x01):
                    print("Pitch")
                    self.pitch_angle = set_angle
                elif(ord(data[5]) == 0x02):
                    print("Roll")
                    self.roll_angle = set_angle
                elif(ord(data[5]) == 0x03):
                    print("Yaw")
                    self.yaw_angle = set_angle
                control_ardusub.press_release_buttons(control_ardusub.master,[3,5],self.x,self.y,self.z,self.r)
                print(set_angle)

                ardusub_connect.set_target_attitude(self.roll_angle-180,self.pitch_angle-180,self.yaw_angle)
        

    def send(self):

       
        while True:
            
            msg = control_ardusub.master.recv_match(blocking=True)

            if(msg.get_type() == 'ATTITUDE'):
                
                self.pitch = msg.pitch*180/math.pi+360
                self.roll = msg.roll*180/math.pi+360
                self.yaw = (msg.yaw*180/math.pi+360)%360

            elif(msg.get_type() == 'VFR_HUD'):
                
                self.depth = msg.alt*100
                self.depth = abs(self.depth)
                #print('Depth:', depth)
            elif(msg.get_type() == 'SCALED_PRESSURE2'):
                self.tempture = msg.temperature
                #print('temperature:', msg.temperature/100)
            elif(msg.get_type() == 'HEARTBEAT'):
                if(msg.base_mode == 209):
                    self.status=1
                elif(msg.base_mode == 81):
                    self.status=0

    def send_click(self):

        send_data = bytearray()
        data_pitch = bytearray()
        data_roll = bytearray()
        data_yaw = bytearray()
        data_depth = bytearray()
        data_tempture = bytearray()


        data_pitch = struct.pack(">H", int(self.pitch*100))
        data_roll = struct.pack(">H", int(self.roll*100))
        data_yaw = struct.pack(">H", int(self.yaw*100))
        data_depth = struct.pack(">H", int(self.depth))
        data_tempture = struct.pack(">H", int(self.tempture))





        data_head = [0xfb,0xfa,0xf9,0xf8,0x15,0x0f]
        data_end = [0x05,0x06,0x07,0x08]



        for num in range(len(data_head)):
            send_data.append(data_head[num])

        for num in range(len(data_pitch)):
            send_data.append(data_pitch[num])

        for num in range(len(data_roll)):
            send_data.append(data_roll[num])

        for num in range(len(data_yaw)):
            send_data.append(data_yaw[num])
        
        
        for num in range(len(data_depth)):
            send_data.append(data_depth[num])

        for num in range(len(data_tempture)):
            send_data.append(data_tempture[num])

        send_data.append(self.status)

        for num in range(len(data_end)):
            send_data.append(data_end[num])
        
        self.sock.sendto(send_data, self.server_address)
        #print("pitch:",self.pitch-360,"roll:",self.roll-360,"yaw:",self.yaw%360,"depth:",self.depth,"tempture:",self.tempture)