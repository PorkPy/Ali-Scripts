#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('/gripper/gripper_status', Bool, queue_size=10)
    rospy.init_node('gripper_status_pub', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        msg=True
        pub.publish(msg)
        rate.sleep()
        #pub.publish(not(msg))


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
