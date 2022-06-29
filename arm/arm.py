# coding=UTF-8

'''====================================================================================
机御科技的机械臂功能
基于串口舵机的机械臂
===================================================================================='''


import sys
import servo                                                # 导入机御科技的舵机库


'''
机械臂本体控制

    - list            转角列表[]
'''
def control(list) :
    # 角度值转换为 0-4096
    for i in range(5):
        list[i] = int(list[i]*12.33333 + 2048)

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
终端指令测试
===================================================================================='''

# $ python arm.py control <angle1> <angle2> <angle3> <angle4> <angle5>
# <angle1> 单位为度，直立状态为0，向前和左为正，向后右为负，
if sys.argv[1] == 'control' and len(sys.argv)==7 :
    dg1 = int(sys.argv[2])
    dg2 = int(sys.argv[3])
    dg3 = int(sys.argv[4])
    dg4 = int(sys.argv[5])
    dg5 = int(sys.argv[6])
    degree_list = [dg1, dg2, dg3, dg4, dg5]
    control(degree_list)




'''====================================================================================
用于兼容亚博的机械臂指令
===================================================================================='''
def Arm_serial_servo_write6(dg1, dg2, dg3, dg4, dg5, dg6, stime) :
    
    list = [dg1, dg2, dg3, dg4, dg5]
    
    # 角度值转换为 0-4096
    for i in range(5):
        list[i] = int(list[i]*11.3777 + 1024)       # 中位90度-2048

    servo.set5(list,stime)



def Arm_serial_servo_read(sid) :
    angle = servo.get1(sid)
    












