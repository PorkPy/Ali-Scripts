#!/usr/bin/env python
import numpy as np
import rospy
import math
import sys
import copy
import moveit_commander
import moveit_msgs
import scipy
from scipy.integrate import odeint
from scipy import signal
from sensor_msgs.msg import JointState
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from rospy.numpy_msg import numpy_msg
#from std_msgs.msg import Floats
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import std_msgs
import sys, time
import jsonlib
import message_filters
from std_msgs.msg import Int8

def Posepublisher(pattern):
    print 'Initialising Publishers'
    rospy.init_node('Manipulate_PubTest', anonymous=True)
    #self.dispenser_status = rospy.Subscriber("/dispenser/dispenser_status",Bool,self.callback)
    targetPosePub=rospy.Publisher('/Handle/TargetPattern', Int8, queue_size=10)
    roboSpeedPub=rospy.Publisher('/Handle/Speed', Float32, queue_size=10)
    triggerPub=rospy.Publisher('/Handle/Trigger',Bool,queue_size=10)
    gripperPub=rospy.Publisher('/gripper/gripper_status',Bool,queue_size=10)
    rate = rospy.Rate(1) # 10hz
    #def publish_Pose:
    Published=False

    closeGrip=Bool()
    closeGrip.data=True

    while not rospy.is_shutdown():
        #print 'Publish ...'
        gripperPub.publish(closeGrip)
        patternMsg=Int8()
        patternMsg.data=pattern
        targetPosePub.publish(patternMsg)
        #print pose_target
        speed=Float32()
        speed.data=10
        roboSpeedPub.publish(speed)
        trigger=Bool()
        trigger.data=True
        triggerPub.publish(trigger)


#def main(args):


if __name__ == '__main__':


    try:
        Posepublisher(2)
    except rospy.ROSInterruptException:
        pass
