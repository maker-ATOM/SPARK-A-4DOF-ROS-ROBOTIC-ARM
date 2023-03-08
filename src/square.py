#!/usr/bin/env python3

# draws a sqaure with vertex [[0.5,0.5,2.36], [1,0.5,2.6], [1,1,1.75], [0.5,1,2.2]]

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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

    return[q0, q1, q2]

def sendTo(x, y, phi):
        gotoPose = IK(x, y, phi)
        gotoPose.insert(0,0)

        trajectory_msg.points[0].positions = gotoPose

        rospy.sleep(cusdelay)

        trajectory_publihser.publish(trajectory_msg)




if __name__ == '__main__':      # main function


    #---------------------------------------VARIABLES------------------------------------------#

    cusdelay = 0.05

    wayPoints = [[0.5,0.5,2.36] ,
                 [1,0.5,2.6]    ,
                 [1,1,1.75]     ,
                 [0.5,1,2.2]    ,
                 [0.5,0.5,2.36] 
                 ]
    
    stepsize = 0.02

    #----------------------------------------SETUP---------------------------------------------#

    rospy.init_node('arm_trajectory_publisher')
    contoller_name = '/arm_controller/command'
    trajectory_publihser = rospy.Publisher(
        contoller_name, JointTrajectory, queue_size=10)

    arm_joints = ['Rev1', 'Rev2', 'Rev3','Rev4']
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = arm_joints
    trajectory_msg.points.append(JointTrajectoryPoint())

    # set the velocity and accel only once
    trajectory_msg.points[0].velocities = [0.0 for i in arm_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in arm_joints]

    # time in which the arm reaches the endpoint
    trajectory_msg.points[0].time_from_start = rospy.Duration(cusdelay + 0.1)


    #----------------------------------------LOOP----------------------------------------------#

    while True:
        x = wayPoints[0][0]
        y = wayPoints[0][1]
        phi = wayPoints[0][2]

        while x < 1:
            sendTo(x, y, phi)
            x = x + stepsize
            phi = phi + 0.0096

        x = wayPoints[1][0]
        y = wayPoints[1][1]
        phi = wayPoints[1][2]

        while y < 1:
            sendTo(x, y, phi)
            y = y + stepsize
            phi = phi - 0.034

        x = wayPoints[2][0]
        y = wayPoints[2][1]
        phi = wayPoints[2][2]

        while x > 0.5:
            sendTo(x, y, phi)
            x = x - stepsize
            phi = phi + 0.018

        x = wayPoints[3][0]
        y = wayPoints[3][1]
        phi = wayPoints[3][2]

        while y > 0.5:
            sendTo(x, y, phi)
            y = y - stepsize
            phi = phi + 0.0064
