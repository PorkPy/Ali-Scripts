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

def Posepublisher(init_pose):
    print 'Initialising Publishers'
    rospy.init_node('Manipulate_PubTest', anonymous=True)
    #self.dispenser_status = rospy.Subscriber("/dispenser/dispenser_status",Bool,self.callback)
    targetPosePub=rospy.Publisher('/Manipulate/TargetPose', PoseStamped, queue_size=10)
    roboSpeedPub=rospy.Publisher('/Manipulate/Speed', Float32, queue_size=10)
    triggerPub=rospy.Publisher('/Manipulate/Trigger',Bool,queue_size=10)
    gripperPub=rospy.Publisher('/gripper/gripper_status',Bool,queue_size=10)
    rate = rospy.Rate(1) # 10hz
    #def publish_Pose:
    Published=False
    i=0
    closeGrip=Bool()
    closeGrip.data=True
    gripperPub.publish(closeGrip)
    while not rospy.is_shutdown() and not(Published):
        #print 'Publish ...'
        pose_target = PoseStamped()
        pose_target.header.stamp=rospy.Time.now()
        pose_target.header.seq+=1
        pose_target.pose.position.x = init_pose[0]
        pose_target.pose.position.y = init_pose[1]
        pose_target.pose.position.z = init_pose[2]
        pose_target.pose.orientation.x =init_pose[3]
        pose_target.pose.orientation.y =init_pose[4]
        pose_target.pose.orientation.z =init_pose[5]
        pose_target.pose.orientation.w =init_pose[6]
        targetPosePub.publish(pose_target)
        print pose_target
        speed=Float32()
        speed.data=10
        roboSpeedPub.publish(speed)
        trigger=Bool()
        trigger.data=True
        triggerPub.publish(trigger)

        if i>50:
            Published=True
            i+=1

#def main(args):


if __name__ == '__main__':
    init_pose=np.array([0.41, -0.031, 0.481079, -0.63913, -0.000138344988, -0.00046901650,0.768946])

    try:
        Posepublisher(init_pose)
    except rospy.ROSInterruptException:
        pass
