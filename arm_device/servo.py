# coding=UTF-8


'''====================================================================================
机御科技的舵机驱动
===================================================================================='''

import sys
import serial
import time


uart_dev = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)

'''
设置单个舵机角度

    - sid: (servo id)       int 0-255       关节id        
    - sdg: (servo degree)   int 0-4096      关节角度        
    - stime: (servo time)   int 0-inf ms    执行时长    
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

    - dg_list: (len = 5)    list    0-4096      包含5个角度值的列表，
    - stime: (servo time)   int     0-inf ms    执行时长
'''
def set5(dg_list, stime):
    dg1 = dg_list[0]
    dg2 = dg_list[1]
    dg3 = dg_list[2]
    dg4 = dg_list[3]
    dg5 = dg_list[4]
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


'''
转矩开关

    - ctrl: (1 or 0)       0 关闭转矩    1 打开转矩
'''
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


'''
查询1个舵机的中位偏移量

    - sid: (servo id)       关节id
    - return int            转角
'''
def offset_read1(sid) :
    data_body_id        = [sid]                             # 舵机ID
    data_body_type      = [0x02]                            # 指令类型
    data_body_parameter = [0x14, 0x02]                      # 参数
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
            offset  = (rx_data_frame[5]<<8) + \
                    rx_data_frame[6]                        # 数据拼接计算
            return offset
        else :                                              # 求和校验未通过   @@@
            print("校验错误\n")
            return -1
    else :                                                  # 帧头检查未通过   !!!
        print("帧头错误\n")
        return -2

'''
设置1个舵机的中位偏移量

    - sid: (servo id)       关节id
    - value:            偏移值
'''
def offset_set1(sid,value) :
    data_body_id        = [sid]                             # 舵机ID
    data_body_type      = [0x03]                            # 指令类型
    data_body_parameter = [0x14]                            # 参数
    data_body_parameter +=[(value>>8)&0xff, value&0xff]
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






