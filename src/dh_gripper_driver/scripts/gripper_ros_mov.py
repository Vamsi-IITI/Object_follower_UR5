#!/usr/bin/env python
import rospy
from dh_gripper_msgs.msg import GripperCtrl

def talker():
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    rospy.init_node('gripper', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = GripperCtrl()
    msg.initialize = False
    while True:

        msg.position = input("Enter Position")
        msg.force = input("Enter Force")
        msg.speed = 100

        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
