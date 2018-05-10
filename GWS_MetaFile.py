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
from geometry_msgs.msg import Pose
from rospy.numpy_msg import numpy_msg
#from std_msgs.msg import Floats
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import std_msgs
import sys, time
import jsonlib
import message_filters

##Gluing workstation GWS
####################################
# Q/dletaP (s)= (A_n)/(ro*L_n*s+32(K*L_n)/D_n^2)
#
# For simplicity:
#
#
#
#########################################################################

#
# class GWS:
#     def _init_(self,product_name,product_ID,recipe):
#         self.recipeName=recipe_Name
#         self.productID=product_ID
#         self.productName=product_Name
#         self.g={'Product name':product_name,'product ID':product_ID,'recipe':recipe}
#         print 'Recipe initilalised:\n'
#         print self.g
#
#     def save_recipe(self,g,recipeName):
#         josnlib.dump('GWS_'+self.recipeName)
#         print 'Recipe saved'
#class Robot(GWS):
class irb120:
    def __init__(self):
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        print 'Initialising Gluing Work station'
        print '================================='
        curPose=self.group.get_current_pose()
        print 'Robot current pose: ',curPose
        print 'dispensing status: '+ 'Off'
        #self._vicon_sub=rospy.Subscriber('/vicon/peg_1/peg_1',geometry_msgs.msg.TransformStamped,self.callback)
        self._speed_=  message_filters.Subscriber('/Manipulate/Speed',Float32)
        self._robotKPIs_= rospy.Publisher('/Manipulate/KPIs',std_msgs.msg.Float32MultiArray,queue_size=100)
        #self._GWS_status_=rospy.Publisher('/GWS_status',Bool,queue_size=100)
        self._InMotion_=rospy.Publisher('/Manipulate/InMotion',Bool,queue_size=100)

        self._rpose_sub=message_filters.Subscriber('/Manipulate/TargetPose',PoseStamped)
        self._trigger_sub=message_filters.Subscriber('/Manipulate/Trigger',Bool)
        self._ts = message_filters.ApproximateTimeSynchronizer([self._rpose_sub,self._speed_,self._trigger_sub],10,10,10)
        self._ts.registerCallback(self.callback)
        moveit_commander.roscpp_initialize(sys.argv)

        #rospy.init_node('Joints_publisher', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        rate = rospy.Rate(1)
        print 'register the callback ....'

    def callback(self,goalPose,Rspeed,trigger):
        '''
        This function return a trajectory between initial pose and goal pose (manipulation skil).

        '''
        def get_kinetic_energy(initial_point,target,Ttime):
            Wp_mass=0.015#Kg
            distance=((target[0]-initial_point.position.x)**2+(target[1]-initial_point.position.y)**2+(target[2]-initial_point.position.z)**2)**0.5
            speed=distance/Ttime
            Ken=0.5*Wp_mass*speed**2
            return Ken

        #kpis={'execution time':0,'energy':0}
        KPIS=std_msgs.msg.Float32MultiArray()
        KPIS.layout.dim=2
        KPIS.data=[0,0]
        #if verbose:
        #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
        currentPose=self.group.get_current_pose().pose
        print "============ Printing robot state"
        print currentPose
        print "============"

        self.group.clear_pose_targets()
        self.group_variable_values = self.group.get_current_joint_values()
        #initialise target position
        pose_target =Pose()
        pose_target.position.x = goalPose.pose.position.x
        pose_target.position.y = goalPose.pose.position.y
        pose_target.position.z = goalPose.pose.position.z
        pose_target.orientation.x =goalPose.pose.orientation.x
        pose_target.orientation.y =goalPose.pose.orientation.y
        pose_target.orientation.z =goalPose.pose.orientation.z
        pose_target.orientation.w =goalPose.pose.orientation.w
        self.group.set_pose_target(pose_target)
        plan1 = self.group.plan()

        # Visualisation in RVIZ
        #if verbose:
            #print "============ Visualizing plan1"
            #display_trajectory = moveit_msgs.Triggermsg.DisplayTrajectory()
            #display_trajectory.trajectory_start = robot.get_current_state()
            #display_trajectory.trajectory.append(plan1)
            #display_trajectory_publisher.publish(display_trajectory)
            #print "============ Waiting while plan1 is visualized (again)..."
            #rospy.sleep(5)

        goal_reached=False
        InMotionBool=Bool()
        InMotionBool.data=goal_reached
        self._InMotion_.publish(InMotionBool)

        if  np.logical_and(not(goal_reached),  trigger):
            startTime=time.clock()
            curPose=self.group.get_current_pose()
            #print type(group.get_current_pose())
            curPose_np=(np.array([curPose.pose.position.x,curPose.pose.position.y,curPose.pose.position.z,curPose.pose.orientation.x,curPose.pose.orientation.y,curPose.pose.orientation.z,curPose.pose.orientation.w]))
            goalPose_np=(np.array([goalPose.pose.position.x,goalPose.pose.position.y,goalPose.pose.position.z,goalPose.pose.orientation.x,goalPose.pose.orientation.y,goalPose.pose.orientation.z,goalPose.pose.orientation.w]))
            print 'target: ',pose_target

            goal_reached= np.all(np.isclose(curPose_np,goalPose_np,atol=1e-3))
            #Publish robot status: Ready: (True/False)

            self.group.go(wait=True)
            InMotionBool=Bool()
            InMotionBool.data=goal_reached
            self._InMotion_.publish(InMotionBool)

        executionTime=time.clock()-startTime


        KPIS.data[0]=executionTime
        KPIS.data[1]=get_kinetic_energy(currentPose,goalPose_np,executionTime)
        self._robotKPIs_.publish(KPIS)


        #print 'Skill execution: Done!\n'
        #print kpis

        #return kpis


def main(args):
    #init_pose=np.array([0.63, 0.081, -0.142, -0.651, -0.0314, -0.00945,0.76])
    #print 'here'
    rospy.init_node('Manipulate', anonymous=True)
    #goal_pose=np.array([0.41, -0.031,s 0.4811, -0.65639, -0.000255309, 0.0002315,0.75442])
    #robotSpeed=10
    ABB=irb120()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        passs


if __name__ == '__main__':
    main(sys.argv)
