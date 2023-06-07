#!/usr/bin/env python
import rospy
from dh_gripper_msgs.msg import GripperCtrl

def talker():
    pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    rospy.init_node('gripper', anonymous=True)
    r = rospy.Rate(10) #10hz
    msg = GripperCtrl()
    msg.initialize = True
    msg.position = 0
    msg.force = 100
    msg.speed = 100

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
