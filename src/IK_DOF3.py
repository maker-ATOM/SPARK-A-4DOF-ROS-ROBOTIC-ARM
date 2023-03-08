#!/usr/bin/env python3

# In this code IKs are solved only for 2D plane that is for Rev2, 3 and 4
# considering 3 link robot
# pose data is taken from terminal

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import math

def IK(x, y, phi):
    l0 = 0.75
    l1 = 0.75
    l2 = 0.25

    xd = x - l2 * math.sin(phi)
    yd = y - l2 * math.cos(phi)

    q1 = math.acos(( xd**2 + yd**2 - l0**2 - l1**2 ) / (2*l0*l1))
    q0 = 1.57 - math.atan(yd/xd) - math.atan((l1 * math.sin(q1)) / ( l0 + l1 * math.cos(q1)))
    q2 = phi - q0 - q1

    # angles in radians
    print("q0 = %.2f " % q0, "q1 = %.2f " % q1, "q2 = %.2f " % q2)

    return[q0, q1, q2]

def perform_trajectory():
    rospy.init_node('arm_trajectory_publisher')
    contoller_name = '/arm_controller/command'
    trajectory_publihser = rospy.Publisher(
        contoller_name, JointTrajectory, queue_size=10)
    argv = sys.argv[1:]
    arm_joints = ['Rev1', 'Rev2', 'Rev3', 'Rev4']

    #  read [x, y, theta]
    goto_pose = [float(argv[0]), float(argv[1]), float(argv[2])]

    # inverse kinematics solution for received inputs
    solved_pose = IK(goto_pose[0], goto_pose[1], goto_pose[2])
    solved_pose.insert(0,0)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = arm_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = solved_pose
    trajectory_msg.points[0].velocities = [0.0 for i in arm_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in arm_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(2)
    rospy.sleep(1)

    trajectory_publihser.publish(trajectory_msg)


if __name__ == '__main__':
    perform_trajectory()
