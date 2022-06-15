# coding=UTF-8

'''====================================================================================
机御科技的机械臂功能
===================================================================================='''


import sys
import servo                                                # 导入机御科技的舵机库


'''
机械臂本体控制

    - list            转角列表
'''
def control(list) :
    # 角度值转换为 0-4096
    for i in range(5):
        list[i] = int(list[i]*11.3777 + 2048)

    # 运行时间计算
    current_degree = servo.get()
    max_err = 0;
    for i in range(5):
        if (abs(current_degree[i] - list[i])>max_err):
            max_err = abs(current_degree[i] - list[i])
    run_time = max_err                                      # 可以乘除控制运行速度

    servo.set5(list,run_time)


'''
机械臂夹手控制

    - list            转角列表
'''
def gripper(degree) :
    servo.set1(6,degree)


'''
机械臂夹手控制

    - list            转角列表
'''
def torque(ctrl) :
    servo.torque(ctrl)




'''====================================================================================
用于兼容亚博的机械臂指令
===================================================================================='''
def Arm_serial_servo_write6(dg1, dg2, dg3, dg4, dg5, dg6, stime) :
    
    list = [dg1, dg2, dg3, dg4, dg5]
    
    # 角度值转换为 0-4096
    for i in range(5):
        list[i] = int(list[i]*11.3777 + 1024)       # 中位90度-2048

    servo.set5(list,stime)













