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
from std_msgs.msg import Int8
#from std_msgs.msg import Floats
from std_msgs.msg import Float32
from std_msgs.msg import Bool
import std_msgs
import sys, time
import jsonlib
import collections
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

NewPattern=collections.deque([],2)
NewPattern.append(0)

class irb120:
    def __init__(self):

        self.NewPatternRecieved=True
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        print 'Initialising Gluing Work station'
        print '================================='
        curPose=self.group.get_current_pose()
        print 'Robot current pose: ',curPose
        print 'dispensing status: '+ 'Off'
        #self._vicon_sub=rospy.Subscriber('/vicon/peg_1/peg_1',geometry_msgs.msg.TransformStamped,self.callback)
        self._speed_=  message_filters.Subscriber('/Handle/Speed',Float32)
        self._robotKPIs_= rospy.Publisher('/Handle/KPIs',std_msgs.msg.Float32MultiArray,queue_size=100)
        #self._GWS_status_=rospy.Publisher('/GWS_status',Bool,queue_size=100)
        self._InMotion_=rospy.Publisher('/Handle/InMotion',Bool,queue_size=100)

        self._pattern_sub=message_filters.Subscriber('/Handle/TargetPattern',Int8)
        self._trigger_sub=message_filters.Subscriber('/Handle/Trigger',Bool)
        self._ts = message_filters.ApproximateTimeSynchronizer([self._pattern_sub,self._speed_,self._trigger_sub],10,10,10)
        self._ts.registerCallback(self.callback)
        moveit_commander.roscpp_initialize(sys.argv)

        #rospy.init_node('Joints_publisher', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        rate = rospy.Rate(1)
        print 'register the callback ....'

    def callback(self,pattern,Rspeed,trigger):
        '''
        This function return a trajectory between initial pose and goal pose (manipulation skil).

        '''
        global NewPattern
        NewPattern.append(pattern.data)
        raster=np.array([[0.41, -0.031, 0.481079, -0.63913, -0.000138344988, -0.00046901650,0.768946],
        [0.41, -0.031-5.5*1e-2, 0.481079, -0.63913, -0.000138344988, -0.00046901650,0.768946],
    [0.41-1.3*1e-2, -0.031-5.5*1e-2, 0.481079, -0.63913, -0.000138344988, -0.00046901650,0.768946],
    [0.41-1.3*1e-2, -0.031, 0.481079, -0.63913, -0.000138344988, -0.00046901650,0.768946],
    [0.41-2*1.3*1e-2, -0.031, 0.481079, -0.63913, -0.000138344988, -0.00046901650,0.768946],
    [0.41-2*1.3*1e-2, -0.031-5.5*1e-2, 0.481079, -0.63913, -0.000138344988, -0.00046901650,0.768946]])

        def get_kinetic_energy(initial_point,target,Ttime):
            Wp_mass=0.015#Kg
            distance=((target[0]-initial_point.position.x)**2+(target[1]-initial_point.position.y)**2+(target[2]-initial_point.position.z)**2)**0.5
            speed=distance/Ttime
            Ken=0.5*Wp_mass*speed**2
            return Ken

        def MOVE(goalPose,goal_reached,trigger,plan):
            if  not(goal_reached) and trigger:
                startTime=time.clock()
                curPose=self.group.get_current_pose()
                #print type(group.get_current_pose())
                curPose_np=(np.array([curPose.pose.position.x,curPose.pose.position.y,curPose.pose.position.z,curPose.pose.orientation.x,curPose.pose.orientation.y,curPose.pose.orientation.z,curPose.pose.orientation.w]))
                #goalPose_np=(np.array([goalPose.pose.position.x,goalPose.pose.position.y,goalPose.pose.position.z,goalPose.pose.orientation.x,goalPose.pose.orientation.y,goalPose.pose.orientation.z,goalPose.pose.orientation.w]))
                goal_reached= np.all(np.isclose(curPose_np,goalPose,atol=1e-3))
                print 'target: ',pose_target


                #Publish robot status: Ready: (True/False)

                self.group.go(wait=True)


            return goal_reached,time.clock()-startTime,get_kinetic_energy(currentPose,goalPose,executionTime)

        executionTime=0
        energy=0
        KPIS=std_msgs.msg.Float32MultiArray()
        KPIS.layout.dim=2
        KPIS.data=[0,0]
        self.NewPatternRecieved=not(NewPattern[0]==NewPattern[1])
        if pattern.data==1 and self.NewPatternRecieved:
            for i in range(4):
                #kpis={'execution time':0,'energy':0}
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
                if i==0:
                    goalPose=raster[1,:]
                    pose_target.position.x = raster[1,0]
                    pose_target.position.y = raster[1,1]
                    pose_target.position.z = raster[1,2]
                    pose_target.orientation.x =raster[1,3]
                    pose_target.orientation.y =raster[1,4]
                    pose_target.orientation.z =raster[1,5]
                    pose_target.orientation.w =raster[1,6]
                    self.group.set_pose_target(pose_target)
                    plan1 = self.group.plan()


                    goal_reached=False
                    InMotionBool=Bool()
                    InMotionBool.data=goal_reached
                    self._InMotion_.publish(InMotionBool)
                    _,executionTime,energy=MOVE(goalPose,goal_reached,trigger,plan1)

                    executionTime+=executionTime
                    energy+=energy

                elif i==1:
                    goalPose=raster[5,:]
                    pose_target.position.x = raster[5,0]
                    pose_target.position.y = raster[5,1]
                    pose_target.position.z = raster[5,2]
                    pose_target.orientation.x =raster[5,3]
                    pose_target.orientation.y =raster[5,4]
                    pose_target.orientation.z =raster[5,5]
                    pose_target.orientation.w =raster[5,6]
                    self.group.set_pose_target(pose_target)
                    plan1 = self.group.plan()

                    goal_reached=False
                    InMotionBool=Bool()
                    InMotionBool.data=goal_reached
                    self._InMotion_.publish(InMotionBool)
                    _,executionTime,energy=MOVE(goalPose,goal_reached,trigger,plan1)
                    executionTime+=executionTime
                    energy+=energy

                elif i==2:
                    goalPose=raster[4,:]
                    pose_target.position.x = raster[4,0]
                    pose_target.position.y = raster[4,1]
                    pose_target.position.z = raster[4,2]
                    pose_target.orientation.x =raster[4,3]
                    pose_target.orientation.y =raster[4,4]
                    pose_target.orientation.z =raster[4,5]
                    pose_target.orientation.w =raster[4,6]
                    self.group.set_pose_target(pose_target)
                    plan1 = self.group.plan()

                    goal_reached=False
                    InMotionBool=Bool()
                    InMotionBool.data=goal_reached
                    self._InMotion_.publish(InMotionBool)
                    _,executionTime,energy=MOVE(goalPose,goal_reached,trigger,plan1)
                    executionTime+=executionTime
                    energy+=energy

                elif i==3:
                    goalPose=raster[0,:]
                    pose_target.position.x = raster[0,0]
                    pose_target.position.y = raster[0,1]
                    pose_target.position.z = raster[0,2]
                    pose_target.orientation.x =raster[0,3]
                    pose_target.orientation.y =raster[0,4]
                    pose_target.orientation.z =raster[0,5]
                    pose_target.orientation.w =raster[0,6]
                    self.group.set_pose_target(pose_target)
                    plan1 = self.group.plan()

                    goal_reached=False
                    InMotionBool=Bool()
                    InMotionBool.data=goal_reached
                    self._InMotion_.publish(InMotionBool)
                    goal_reached,executionTime,energy=MOVE(goalPose,goal_reached,trigger,plan1)
                    executionTime+=executionTime
                    energy+=energy
                    InMotionBool.data=goal_reached
                    self._InMotion_.publish(InMotionBool)


        elif pattern.data==2 and self.NewPatternRecieved:
            for i in range(6):
                goalPose=raster[i,:]
                #kpis={'execution time':0,'energy':0}
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
                pose_target.position.x = raster[i,0]
                pose_target.position.y = raster[i,1]
                pose_target.position.z = raster[i,2]
                pose_target.orientation.x =raster[i,3]
                pose_target.orientation.y =raster[i,4]
                pose_target.orientation.z =raster[i,5]
                pose_target.orientation.w =raster[i,6]
                self.group.set_pose_target(pose_target)
                plan1 = self.group.plan()

                goal_reached=False
                InMotionBool=Bool()
                InMotionBool.data=goal_reached
                self._InMotion_.publish(InMotionBool)
                goal_reached,executionTime,energy=MOVE(goalPose,goal_reached,trigger,plan1)
                executionTime+=executionTime
                energy+=energy
                if i==5:
                    InMotionBool.data=True
                    self._InMotion_.publish(InMotionBool)
                    break


        elif pattern.data==3 and self.NewPatternRecieved:

            #kpis={'execution time':0,'energy':0}
            #if verbose:
            #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
            currentPose=self.group.get_current_pose().pose
            print "============ Printing robot state"
            print currentPose
            print "============"

            self.group.clear_pose_targets()
            self.group_variable_values = self.group.get_current_joint_values()
            #initialise target position
            goalPose=raster[0,:]
            pose_target =Pose()
            pose_target.position.x = raster[0,0]
            pose_target.position.y = raster[0,1]
            pose_target.position.z = raster[0,2]
            pose_target.orientation.x =raster[0,3]
            pose_target.orientation.y =raster[0,4]
            pose_target.orientation.z =raster[0,5]
            pose_target.orientation.w =raster[0,6]
            self.group.set_pose_target(pose_target)
            plan1 = self.group.plan()

            goal_reached=False
            InMotionBool=Bool()
            InMotionBool.data=goal_reached
            self._InMotion_.publish(InMotionBool)
            goal_reached,executionTime,energy=MOVE(goalPose,goal_reached,trigger,plan1)
            executionTime+=executionTime
            energy+=energy

        elif pattern.data==4 and self.NewPatternRecieved:

            #kpis={'execution time':0,'energy':0}
            #if verbose:
            #display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
            currentPose=self.group.get_current_pose().pose
            print "============ Printing robot state"
            print currentPose
            print "============"
            PickUpPose=np.array([0.384353593373    , 0.0624897707675, 0.281119940958      , -0.639059206099    , 0.000164098531592    , -0.00081550848792    , 0.769157096514])
            self.group.clear_pose_targets()
            self.group_variable_values = self.group.get_current_joint_values()
            #initialise target position
            goalPose=PickUpPose
            pose_target =Pose()
            pose_target.position.x = PickUpPose[0]
            pose_target.position.y = PickUpPose[1]
            pose_target.position.z = PickUpPose[2]
            pose_target.orientation.x =PickUpPose[3]
            pose_target.orientation.y =PickUpPose[4]
            pose_target.orientation.z =PickUpPose[5]
            pose_target.orientation.w =PickUpPose[6]
            self.group.set_pose_target(pose_target)
            plan1 = self.group.plan()

            goal_reached=False
            InMotionBool=Bool()
            InMotionBool.data=goal_reached
            self._InMotion_.publish(InMotionBool)
            goal_reached,executionTime,energy=MOVE(PickUpPose,goal_reached,trigger,plan1)
            executionTime+=executionTime
            energy+=energy
            InMotionBool.data=goal_reached
            self._InMotion_.publish(InMotionBool)
###########################################################################


        KPIS.data[0]=executionTime
        KPIS.data[1]=energy
        self._robotKPIs_.publish(KPIS)


                #print 'Skill execution: Done!\n'
                #print kpis

                #return kpis


def main(args):
    #init_pose=np.array([0.63, 0.081, -0.142, -0.651, -0.0314, -0.00945,0.76])
    #print 'here'
    rospy.init_node('Handle_skills', anonymous=True)
    #goal_pose=np.array([0.41, -0.031,s 0.4811, -0.65639, -0.000255309, 0.0002315,0.75442])
    #robotSpeed=10
    ABB=irb120()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        passs


if __name__ == '__main__':
    main(sys.argv)
