# -*- coding: UTF-8 -*-
import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
from typing import Union, Iterable


from typing import Union, Iterable
from pymavlink import mavutil
import time




# from ardusub.com/developers/full-parameter-list.html#btnn-parameters
ARDUSUB_BTN_FUNCTIONS = {
    'disabled': 0,
    'shift': 1,
    'arm_toggle': 2,
    'arm': 3,
    'disarm': 4,
    'mode_manual': 5,
    'mode_stabilize': 6,
    'mode_depth_hold': 7,
    'mode_poshold': 8,
    'mode_auto': 9,
    'mount_center': 21,
    'mount_tilt_up': 22,
    'mount_tilt_down': 23,
    'lights1_brighter': 32,
    'lights1_dimmer': 33,
    'lights2_brighter': 35,
    'lights2_dimmer': 36,
    'gain_inc': 42,
    'gain_dec': 43,
    'input_hold_set':48,
    'servo_1_inc':61,
    'servo_1_dec':62,
    'servo_2_min':68,
    'servo_2_max':69,
    'servo_3_min':73,
    'servo_3_max':74,
    # ... any others you're interested in
}

def set_param(autopilot, name, value, type,
              timeout=1):
    name = name.encode('utf8')
    autopilot.mav.param_set_send(
        autopilot.target_system, autopilot.target_component,
        name, value, type
    )
    
    msg = autopilot.recv_match(type='PARAM_VALUE', blocking=True,
                               timeout=timeout)
    # TODO: retries and/or verification that parameter is correctly set
    return msg

def set_button_function(autopilot, button: int, function: Union[int,str],
                        shifted=False):
    shifted = 'S' if shifted else ''
    param = f'BTN{button}_{shifted}FUNCTION'
    if isinstance(function, str):
        function = ARDUSUB_BTN_FUNCTIONS[function.lower()]
    type = mavutil.mavlink.MAV_PARAM_TYPE_INT8
    return set_param(autopilot, param, function, type)

def send_manual_control(autopilot, x=0, y=0, z=500, 
                        r=0, pressed_buttons: Union[int, Iterable[int]] = 0):
    ''' 
    'pressed_buttons' is either 
        a bit-field with 1 bits for pressed buttons (bit 0 -> button 0), or 
        an iterable specifying which buttons are pressed (e.g. [0,3,4])
    '''
    # 检查pressed_buttons是否为整数类型
    if not isinstance(pressed_buttons, int):
        # convert iterable into bit-field values
        pressed_buttons = sum(1 << button for button in pressed_buttons)
    autopilot.mav.manual_control_send(
        autopilot.target_system,
        x,y, z, r,
        pressed_buttons
    )
    print(pressed_buttons)

def press_release_buttons(autopilot, buttons,x,y,z,r):
    # 调用send_manual_control函数，发送手动控制指令，同时按下指定的按钮
    send_manual_control(autopilot,x,y,z,r, pressed_buttons=buttons)
    # 再次调用send_manual_control函数，发送手动控制指令，同时释放所有按钮
    send_manual_control(autopilot,x,y,z,r, pressed_buttons=0)

def heartbeat_trigetr():
    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, 
                              mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
                              0, 0, 0)



master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')

master.wait_heartbeat()



print('master connected!')

master.mav.request_data_stream_send(
    master.target_system, 
    master.target_component, 
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 
    20,  
    1  
)


set_button_function(master,10, 'lights2_brighter')
set_button_function(master,9, 'lights2_dimmer')

set_button_function(master,13, 'servo_2_min')
set_button_function(master,14, 'servo_2_max')

set_button_function(master,0, 'servo_3_min')
set_button_function(master,3, 'servo_3_max')

set_button_function(master,1, 'servo_1_inc')
set_button_function(master,2, 'servo_1_dec')

set_button_function(master,11, 'mount_tilt_up')
set_button_function(master,12, 'mount_tilt_down')

set_button_function(master,8, 'input_hold_set')

set_button_function(master,5, 'shift')
set_button_function(master,7, 'shift')

set_button_function(master,0, 'mode_manual', shifted=True)
set_button_function(master,1, 'mode_stabilize', shifted=True)
set_button_function(master,3, 'mode_depth_hold', shifted=True)

set_button_function(master,2, 'mount_center', shifted=True)

print("Waiting for the vehicle to arm")
# master.motors_armed_wait()
print('Armed!')


class ardusub_control(object):


    def __init__(self):
           
        print("init ardusub_control")

    boot_time = time.time()

    print("control init ready")






    def set_servo_pwm(self, servo_n, microseconds):
        master.set_servo(servo_n+8, microseconds)
      


    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        # 打印通道ID
        print(channel_id)
        # 打印PWM值
        print(pwm)
        # 检查通道ID是否在有效范围内（1到8）
        if channel_id < 1 or channel_id > 18:
            # 如果通道ID不在有效范围内，打印错误信息并返回
            print("Channel does not exist.")
            return

        # 初始化一个长度为8的列表，每个元素初始值为65535
        rc_channel_values = [65535 for _ in range(8)]
        # 将指定通道ID的PWM值设置为传入的pwm值
        rc_channel_values[channel_id - 1] = pwm
        # 发送RC通道覆盖指令
        master.mav.rc_channels_override_send(
            master.target_system,  # target_system
            master.target_component,  # target_component
            *rc_channel_values)  # RC channel list, in microseconds.



    def set_target_depth(self, depth):


        master.mav.set_position_target_global_int_send(
            int(1e3 * (time.time() - self.boot_time)),  # ms since boot
            master.target_system, master.target_component,
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
            ), lat_int=0, lon_int=0, alt=depth,
            vx=0, vy=0, vz=0,
            afx=0, afy=0, afz=0, yaw=0, yaw_rate=0

        )



    def set_target_attitude(self, roll, pitch, yaw):

        master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)),  
            master.target_system, master.target_component,

            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
            0, 0, 0, 0 
        )


    def look_at(self,tilt, roll=0, pan=0):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            1,
            tilt,
            roll,
            pan,
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

