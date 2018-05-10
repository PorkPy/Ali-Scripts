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
import geometry_msgs.msg
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32
import sys, time
##Dynamic model of dispensing syetem
####################################
# Q/dletaP (s)= (A_n)/(ro*L_n*s+32(K*L_n)/D_n^2)
#
# For simplicity:
#
#
#
#########################################################################

#need to be updated from the masking paper
# (1) Transfer Function


def robot_pose():
    #Joints_pub=rospy.Publisher('/JointState',JointState,queue_size=100)
    Joint0_pub=rospy.Publisher('/Joint0',Float32,queue_size=10)
    Joint1_pub=rospy.Publisher('/Joint1',Float32,queue_size=10)
    Joint2_pub=rospy.Publisher('/Joint2',Float32,queue_size=10)
    Joint3_pub=rospy.Publisher('/Joint3',Float32,queue_size=10)
    Joint4_pub=rospy.Publisher('/Joint4',Float32,queue_size=10)
    Joint5_pub=rospy.Publisher('/Joint5',Float32,queue_size=10)
    #joints=rpspy.Publisher('')

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('Joints_publisher', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander('manipulator')
    rate = rospy.Rate(1)

    group.clear_pose_targets()
    waypoints = []
    group_variable_values = group.get_current_joint_values()
    print group_variable_values
    group_variable_values[0] = 0.14864012002944946
    group.set_joint_value_target(group_variable_values)

    plan2 = group.plan()
    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #
    # display_trajectory.trajectory_start = robot.get_current_state()
    # display_trajectory.trajectory.append(plan2)
    # display_trajectory_publisher.publish(display_trajectory);
    group.go(wait=True)
    print "============ Waiting while RVIZ displays plan2..."
    rospy.sleep(5)



    # start with the current pose
    # Get joints trajectory
    # (plan, fraction) = group.compute_cartesian_path(
    #                        waypoints,   # waypoints to follow
    #                        0.01,        # eef_step
    #                        0.0)         # jump_threshold
    # #moveit_commander.roscpp_shutdown()
    # joints_values=plan.joint_trajectory.points
    # Jointslist=group.get_joints()

    #while not rospy.is_shutdown():
        #group.go(wait=True)
        #for i in range(len(joints_values)):
            #print len(joints_values)

                # Joint0_pub.publish(Joint0_msg)
                # Joint1_pub.publish(Joint1_msg)
                # Joint2_pub.publish(Joint2_msg)
                # Joint3_pub.publish(Joint3_msg)
                # Joint4_pub.publish(Joint4_msg)
                # Joint5_pub.publish(Joint5_msg)
                # rospy.sleep(0.5)

                #rospy.Sleep(5)

if __name__ == '__main__':
    try:
        robot_pose()
    except rospy.ROSInterruptException:
        pass
