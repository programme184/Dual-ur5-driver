#!/usr/bin/env python3
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class UR_control:
    def __init__(self):
        self.default_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_angle_pub = rospy.Publisher('/follow_joint_trajectory/goal',
                                               FollowJointTrajectoryActionGoal,
                                               queue_size=20)
        rospy.sleep(0.5)

    def set_joints(self):
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        now = rospy.get_rostime()
        rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        traj.header.stamp = now
        ag = FollowJointTrajectoryActionGoal()
        ag.goal.trajectory = traj

        touching_item = JointTrajectoryPoint()
        touching_item.positions = [
            math.radians(90),
            math.radians(-82.90),
            math.radians(45),
            math.radians(-107.89),
            math.radians(200),
            math.radians(-38.20)
        ]
        touching_item.velocities = self.default_velocity
        touching_item.time_from_start = rospy.Duration(10.0)
        traj.points.append(touching_item)
        ag = FollowJointTrajectoryActionGoal()
        ag.goal.trajectory = traj

        self.joint_angle_pub.publish(ag)
        rospy.sleep(1)


if __name__ == "__main__":

    rospy.init_node('joints_control', anonymous=True)
    robot = UR_control()
    # while True:
    robot.set_joints()