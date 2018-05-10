#!/usr/bin/env python
import numpy as np
import rospy
import math
import sys
import copy
import moveit_commander
import moveit_msgs
import scipy
import message_filters
from scipy.integrate import odeint
from scipy import signal
from sensor_msgs.msg import JointState
import moveit_msgs.msg
import geometry_msgs.msg
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32
import sys, time
import jsonlib

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

def manipulate(goalPose,Rspeed,verbose=False):
    '''
    This function return a trajectory between initial pose and goal pose (manipulation skil).

    '''
    kpis={'execution time':0,'energy':0}
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Joints_publisher', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander('manipulator')
    rate = rospy.Rate(1)
    if verbose:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)

    print "============ Printing robot state"
    print group.get_current_pose().pose
    print "============"

    group.clear_pose_targets()
    group_variable_values = group.get_current_joint_values()
    #initialise target position
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = goalPose[0]
    pose_target.position.y = goalPose[1]
    pose_target.position.z = goalPose[2]
    pose_target.orientation.x =goalPose[3]
    pose_target.orientation.y =goalPose[4]
    pose_target.orientation.z =goalPose[5]
    pose_target.orientation.w =goalPose[6]
    group.set_pose_target(pose_target)
    plan1 = group.plan()

    # Visualisation in RVIZ
    if verbose:
        print "============ Visualizing plan1"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        display_trajectory_publisher.publish(display_trajectory);

        print "============ Waiting while plan1 is visualized (again)..."
        rospy.sleep(5)

    goal_reached=False
    startTime=time.clock()
    while not rospy.is_shutdown() and not(goal_reached):
        curPose=group.get_current_pose()
        print type(group.get_current_pose())
        curPose_np=(np.array([curPose.pose.position.x,curPose.pose.position.y,curPose.pose.position.z,curPose.pose.orientation.x,curPose.pose.orientation.y,curPose.pose.orientation.z,curPose.pose.orientation.w]))
        print 'target: ',pose_targetac
        goal_reached= np.all(np.isclose(curPose_np,goalPose,atol=1e-3))
        print goal_reached
        group.go(wait=True)
    executionTime=time.clock()-startTime


    kpis['execution time']=executionTime
    print 'Skil execution: Done!\n'
    print kpis

    return kpis


def main(args):
    #init_pose=np.array([0.63, 0.081, -0.142, -0.651, -0.0314, -0.00945,0.76])
    goal_pose=np.array([0.41, -0.031, 0.4811, -0.65639, -0.000255309, 0.0002315,0.75442])
    robotSpeed=10
    try:
        manipulate(goal_pose,robotSpeed)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)
