#! /usr/bin/env python
#--coding:utf-8--
import rospy

from sensor_msgs import JointState

def doMsg(msg):
    rospy.loginfo(msg.position)

if __name__ == "__main__":
    rospy.init_node("jy_arm_driver")
    sub = rospy.Subscriber("sensor_msgs/JointState",JointState,doMsg,queue_size=10)
    rospy.spin()        

