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

class Posepublisher:

    def __init__(self):
        print 'Initialising Publishers'
        self.init_pose=np.array([0.63, 0.081, -0.142, -0.651, -0.0314, -0.00945,0.76])
        rospy.init_node('Manipulate_PubTest', anonymous=True)
        #self.dispenser_status = rospy.Subscriber("/dispenser/dispenser_status",Bool,self.callback)
        self.targetPosePub=rospy.Publisher('/Manipulate/TargetPose', PoseStamped, queue_size=10)
        self.roboSpeedPub=rospy.Publisher('/Manipulate/Speed', Float32, queue_size=10)
        self.triggerPub=rospy.Publisher('/Manipulate/Trigger',Bool,queue_size=10)
        self.Posed_Reached=rospy.Subscriber('/Manipulate/InMotion',Bool,self.callback)
        rate = rospy.Rate(1) # 10hz

    def callback(self,Posed_Reached):
        #def publish_Pose:
        print 'Publish ...'
        pose_target = PoseStamped()
        pose_target.header.stamp=rospy.Time.now()
        pose_target.header.seq+=1
        pose_target.pose.position.x = self.init_pose[0]
        pose_target.pose.position.y = self.init_pose[1]
        pose_target.pose.position.z = self.init_pose[2]
        pose_target.pose.orientation.x =self.init_pose[3]
        pose_target.pose.orientation.y =self.init_pose[4]
        pose_target.pose.orientation.z =self.init_pose[5]
        pose_target.pose.orientation.w =self.init_pose[6]
        self.targetPosePub.publish(pose_target)
        speed=Float32()
        speed.data=10
        self.roboSpeedPub.publish(speed)
        trigger=Bool()
        trigger.data=True
        self.triggerPub.publish(trigger)



def main(args):

    #print 'here
    #goal_pose=np.array([0.41, -0.031,s 0.4811, -0.65639, -0.000255309, 0.0002315,0.75442])
    #robotSpeed=10
    Posepublisher0=Posepublisher()
    try:
        rospy.spin()
        print 'try ...'
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)
