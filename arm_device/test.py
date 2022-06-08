# coding=UTF-8

'''
串口舵机协议的封装，提供上层使用接口
'''

import sys
import serial
import time


# 串口设备控制
uart_dev = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)


def torque(ctrl) :
    data_body_id        = [0xfe]                            # 舵机ID
    data_body_type      = [0x03]                            # 指令类型
    data_body_parameter = [0x28]                            # 参数组1 
    data_body_parameter +=[ctrl]                            # 参数组2 指定关节控制
    data_body_length    = [2 + len(data_body_parameter)]    # 数据长度

    # 组合数据帧
    frame_start  = [0xff, 0xff]                             # 帧头
    frame_body   = data_body_id + data_body_length + \
                data_body_type + data_body_parameter        # 数据体
    i = 0                                                   # 求和校验循环参数
    frame_check = [0x00]
    while i<len(frame_body) :
        frame_check[0] += frame_body[i]
        i += 1
    frame_check[0] = (~frame_check[0])&0xff                 # 求和校验
    data_frame = frame_start + frame_body + frame_check     # 完整数据帧
    
    # 发送一个数据帧
    uart_dev.write(data_frame)    















