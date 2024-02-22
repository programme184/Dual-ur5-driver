#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from util import rad2deg, deg2rad, swap_order
import numpy as np
from myOmni import MyBag  # Import MyBag class
import sys

class MyRobot(MyBag):  # Inherit from MyBag
    def __init__(self):
        self.topic = '/scaled_pos_joint_traj_controller/state'  # Define the topic for reading joint states
        super().__init__(self.topic, filename="ur_joints")  # Initialize the MyBag base class

        # Create a subscriber to the '/joint_states' topic
        self.robot_joint_subscriber = rospy.Subscriber(self.topic, JointTrajectoryControllerState, self.subscriber_callback)
        self.pub = rospy.Publisher('/scaled_pos_joint_traj_controller/command', JointTrajectory, queue_size=10)

        # Initialize joint names and positions
        self.current_joint_positions = []

        # Wait for the subscriber to receive joint names
        rospy.sleep(0.5)

    def subscriber_callback(self, data):
        super().subscriber_callback(data)
        # print(data.actual)
        self.joint_positions = np.array(data.actual.positions)
        self.joint_names = data.joint_names
        
    def get_joints(self):
        return self.joint_positions
    
    def move_joints(self, joint_positions, duration=0.1):
        # Create a JointTrajectory message
        joint_traj = JointTrajectory()

        # Set the joint names from the received message
        joint_traj.joint_names = self.joint_names

        # Create a JointTrajectoryPoint for the desired joint positions
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        print(point.positions)
        # Set the time from start for this point
        point.time_from_start = rospy.Duration(duration)

        # Append the JointTrajectoryPoint to the trajectory
        joint_traj.points.append(point)
        self.pub.publish(joint_traj)  # Call the publish method

def replay_bag(robot, bagname):
    from myBagReader import MyBagReader
    bag_reader = MyBagReader(bagname)
    bag_reader.info()
    # print(bag_reader.info())
    time_interval = 0.1
    data = bag_reader.sample_attribute(topic='/scaled_pos_joint_traj_controller/state', time_interval=time_interval)
    
    print(data)
    for i, joints in enumerate(data):
        print("moving to", joints)
        if i==0: 
            duration=1
        else:
            duration=time_interval
        # robot.move_joints(joints, duration=duration)
        rospy.sleep(duration)
        print('now at', robot.get_joints())
    pass

if __name__ == '__main__':
    try:
        rospy.init_node('ur5_shake_test', anonymous=True)
        robot =  MyRobot()
        replay_bag(robot, sys.argv[1])
    except rospy.ROSInterruptException:
        pass

