# coding=UTF-8

'''
用来在终端里控制舵机的指令
机械臂库使用示例
'''

import sys
import servo                                                # 导入jytech的机械臂库


'''
获得所有舵机的角度
'''
if sys.argv[1] == 'get' :                           
    all_degree = servo.get()                                # 调用
    print(all_degree)

'''
获得1个舵机的角度
'''
if sys.argv[1] == 'get1' and len(sys.argv)==3 :
    servo_id = int(sys.argv[2])
    single_degree = servo.get1(servo_id)                    # 调用
    print(single_degree)

'''
设置1个舵机角度
'''
if sys.argv[1] == 'set1' and len(sys.argv)==5 :
    servo_id = int(sys.argv[2])
    servo_degree = int(sys.argv[3])
    servo_time = int(sys.argv[4])
    servo.set1(servo_id, servo_degree, servo_time)          # 调用

'''
设置5个舵机角度
'''
if sys.argv[1] == 'set5' and len(sys.argv)==8 :
    servo_degree1 = int(sys.argv[2])
    servo_degree2 = int(sys.argv[3])
    servo_degree3 = int(sys.argv[4])
    servo_degree4 = int(sys.argv[5])
    servo_degree5 = int(sys.argv[6])
    servo_time = int(sys.argv[7])
    servo.set5(servo_degree1, servo_degree2, \
                ervo_degree3, servo_degree4, \
                servo_degree5, servo_time)                  # 调用











