# coding=UTF-8

'''
串口舵机协议的封装，提供上层使用接口
'''
import time
import servo


servo.set5([2048, 2048, 2048 , 2048, 2048] , 1000)

time.sleep(2)

servo.set5([2048, 1500, 1500, 1500, 2048], 1000)

time.sleep(2)

servo.set5([1000, 1500, 1500, 1500, 2048], 1000)

time.sleep(1)

servo.set5([3000, 1500, 1500, 1500, 2048], 2500)

time.sleep(3)

servo.set5([3000, 1500, 1500, 2500, 2048], 1500)

time.sleep(3)


servo.set5([3000, 1500, 1500, 1500, 2048], 1000)

time.sleep(1)


servo.set5([3000, 1500, 1500, 2500, 2048], 1000)

time.sleep(1)

servo.set5([3000, 1500, 1500, 1500, 2048], 1000)

time.sleep(1)

servo.set5([3000, 1500, 1500, 2500, 2048], 1000)

time.sleep(1)

