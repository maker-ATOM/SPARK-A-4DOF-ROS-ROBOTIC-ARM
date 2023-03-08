#!/usr/bin/env python3

# forward kinematics in 2D plane, read the angle (in degrees) of joints to be rotated from terminal
# calculates the postion of end effortor and displays it

# TODO
# - Add FK for 3D, include rev1 as well


import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import math

def FK(q0, q1, q2):
    l0 = 0.75
    l1 = 0.75
    l2 = 0.2

    phi = q0 + q1 + q2
    x = l0 * math.sin(q0) + l1 * math.sin(q0 + q1) + l2 * math.sin(q0 + q1 + q2)
    y = l0 * math.cos(q0) + l1 * math.cos(q0 + q1) + l2 * math.cos(q0 + q1 + q2)

    # phi in radians
    print("Arm at: x = %.2f " % x, "y = %.2f " % y, "phi = %.2f " % phi)


def perform_trajectory():
    rospy.init_node('arm_trajectory_publisher')
    contoller_name='/arm_controller/command'
    trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)
    argv = sys.argv[1:]                         
    arm_joints = ['Rev1','Rev2','Rev3','Rev4']
    goal_positions = [ float(argv[0]) , float(argv[1]), float(argv[2]) , float(argv[3])]
    

    # this function can be modified to return a list with FK values
    FK(goal_positions[1], goal_positions[2], goal_positions[3])


    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = arm_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_positions
    trajectory_msg.points[0].velocities = [0.0 for i in arm_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in arm_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(2)
    rospy.sleep(1)
    trajectory_publihser.publish(trajectory_msg)


if __name__ == '__main__':
    perform_trajectory()