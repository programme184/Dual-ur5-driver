#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from myRobot import MyRobot
from util import rad2deg, deg2rad
import numpy as np
import time

def shake(robot, shake_delta):

    joint_degrees = rad2deg(robot.get_joints())
    print(joint_degrees)

    # for i in range(len(joint_degrees)):
    #     for j in [-1, 1]:
    #         joint_degrees[i] += j * shake_delta
    #         joint_positions = deg2rad(joint_degrees)
    #         print('desired rads', joint_positions)
    #         print('current rads', robot.get_joints())
    #         robot.move_joints(joint_positions , duration=0.1)
    #         rospy.sleep(0.5)  # Sleep for 0.5 seconds between movements


if __name__ == '__main__':
    try:
        rospy.init_node('ur5_shake_test', anonymous=True)
        record = False
        robot = MyRobot()  # Initialize the robot object
        if record:
            robot.start_bag_recording()
        # time.sleep(10000)
        shake(robot, shake_delta=2)
        
        if record:
            robot.stop_bag_recording()
    except rospy.ROSInterruptException:
        pass
