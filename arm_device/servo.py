# coding=UTF-8

'''
串口舵机协议的封装，提供上层使用接口
'''

import sys
import serial
import time


# 串口设备控制
uart_dev = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)



'''
设置单个舵机角度
    - sid: (servo id)       关节id
    - sdg: (servo degree)   关节角度
    - stime: (servo time)   执行时长
'''
def set1(sid, sdg, stime):
    data_body_id        = [sid]                             # 舵机ID
    data_body_type      = [0x03]                            # 指令类型
    data_body_parameter = [0x2a]                            # 参数组1 
    data_body_parameter +=[(sdg>>8)&0xff, sdg&0xff, \
                        (stime>>8)&0xff, stime&0xff]        # 参数组2 指定关节控制
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



'''
设置5个舵机角度
    - dgx: (degree x)       id为x的舵机转角
    - stime: (servo time)   执行时长
'''
def set5(dg1, dg2, dg3, dg4, dg5, stime):
    data_body_id        = [0xfe]                            # 舵机ID
    data_body_type      = [0x83]                            # 指令类型
    data_body_parameter = [0x2a, 0x04]                      # 参数组1 
    data_body_parameter +=[0x01, (dg1>>8)&0xff, dg1&0xff,\
                        (stime>>8)&0xff, stime&0xff]        # 参数组2 关节1
    data_body_parameter +=[0x02, (dg2>>8)&0xff, dg2&0xff,\
                        (stime>>8)&0xff, stime&0xff]        # 参数组3 关节2
    data_body_parameter +=[0x03, (dg3>>8)&0xff, dg3&0xff,\
                        (stime>>8)&0xff, stime&0xff]        # 参数组4 关节3
    data_body_parameter +=[0x04, (dg4>>8)&0xff, dg4&0xff,\
                        (stime>>8)&0xff, stime&0xff]        # 参数组5 关节4
    data_body_parameter +=[0x05, (dg5>>8)&0xff, dg5&0xff,\
                        (stime>>8)&0xff, stime&0xff]        # 参数组6 关节5
    data_body_length    = [2 + len(data_body_parameter)]    # 计算数据长度

    # 组合数据帧
    frame_start = [0xff, 0xff]                              # 帧头
    
    frame_body  = data_body_id + data_body_length + \
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




'''
获得单个舵机角度
    - sid: (servo id)       关节id
    - return int            转角
'''
def get1(sid):

    data_body_id        = [sid]                             # 舵机ID
    data_body_type      = [0x02]                            # 指令类型
    data_body_parameter = [0x38, 0x02]                      # 参数组1 
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

    # 接收回应数据
    rx_data = []
    n = 8
    while (uart_dev.inWaiting() != n) :
        time.sleep(0.01)
    rx_data += uart_dev.read(n)

    # 解析回应数据
    i=0
    rx_data_frame = []
    while i<len(rx_data) :
        rx_data_frame = rx_data_frame + [ord(rx_data[i])]   # 转换为10进制数据
        i = i + 1   
    while i<len(rx_data) :
        rx_data_frame = rx_data_frame + [ord(rx_data[i])]   # 转换为10进制数据
        i = i + 1

    if (rx_data_frame[0] == 0xff) and \
        (rx_data_frame[1] == 0xf5) :                        # 帧头检查通过    !!!
        check_sum = (~(rx_data_frame[2] + rx_data_frame[3] +\
                    rx_data_frame[4] + rx_data_frame[5] +\
                    rx_data_frame[6]))&0xff
        if check_sum == rx_data_frame[7] :                  # 求和校验通过    @@@    
            degree  = (rx_data_frame[5]<<8) + \
                    rx_data_frame[6]                        # 数据拼接计算
            return degree
        else :                                              # 求和校验未通过   @@@
            print("校验错误\n")
            return -1
    else :                                                  # 帧头检查未通过   !!!
        print("帧头错误\n")
        return -2



'''
获得所有舵机角度
    - return list            转角列表
'''
def get() :
    degree = [get1(1) ,get1(2) ,get1(3) ,get1(4) , get1(5)]
    return degree








