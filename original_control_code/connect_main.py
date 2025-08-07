# -*- coding: UTF-8 -*-
import udp_connect
import time
import threading



if __name__ == '__main__':
    proxy = udp_connect.UDPProxy('127.0.0.1', 9999, '127.0.0.1', 8888)  # 创建UDP代理对象

    # 向目标服务器发送数据

    while True:
        proxy.send_click()
        time.sleep(0.05)
        
       

       


# 需要取出来的值：
# {'mavpackettype': 'ATTITUDE', 'time_boot_ms': 184432, 'roll': 0.022259538993239403, 'pitch': -0.0662693902850151, 'yaw': 2.720277786254883, 'rollspeed': 0.00281454436480999, 'pitchspeed': -0.00013509691052604467, 'yawspeed': -9.598625911166891e-05}
# {'mavpackettype': 'VFR_HUD', 'airspeed': 0.0, 'groundspeed': 0.0, 'heading': 155, 'throttle': 50, 'alt': -0.029999999329447746, 'climb': -0.05999999865889549}
# {'mavpackettype': 'SCALED_PRESSURE2', 'time_boot_ms': 184712, 'press_abs': 1005.3999633789062, 'press_diff': -0.29999998211860657, 'temperature': 2293}


