#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from util import rad2deg, deg2rad, swap_order
import numpy as np
from myOmni import MyBag  # Import MyBag class
import threading
import sys
import ik_algorithm

class DualRobot(MyBag):  # Inherit from MyBag
    def __init__(self, topic, command):
        self.topic = topic  # Define the topic for reading joint states
        super().__init__(self.topic, filename="ur_joints")  # Initialize the MyBag base class

        # Create a subscriber to the '/joint_states' topic
        self.robot_joint_subscriber = rospy.Subscriber(self.topic, JointTrajectoryControllerState, self.subscriber_callback)
        self.pub = rospy.Publisher(command, JointTrajectory, queue_size=10)

        # Initialize joint names and positions
        self.current_joint_positions = []

        # Wait for the subscriber to receive joint names
        rospy.sleep(0.5)

    def subscriber_callback(self, data):
        super().subscriber_callback(data)
        # print(data.actual)
        self.joint_positions = np.array(data.actual.positions)
        self.joint_names = data.joint_names
        # print(data.joint_names)
        
    def get_joints(self):
        # print('joints:\n', self.joint_positions)
        
        return self.joint_positions
    
    def move_joints(self, joint_positions, duration=0.1):
        # Create a JointTrajectory message
        joint_traj = JointTrajectory()

        # Set the joint names from the received message
        joint_traj.joint_names = self.joint_names

        # Create a JointTrajectoryPoint for the desired joint positions
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        # print(point.positions)
        # Set the time from start for this point
        point.time_from_start = rospy.Duration(duration)

        # Append the JointTrajectoryPoint to the trajectory
        joint_traj.points.append(point)
        self.pub.publish(joint_traj)  # Call the publish method
        rospy.sleep(5)
        
            
        
class MyThread(object):  #自定义一个类
    def __init__(self, func):#, args, name=""
        # self.name = name
        self.func = func
        # self.args = args
    def __call__(self, *args, **kwargs):
        self.func()  #重写类的__call__方法，在该方法内调用要执行的函数     
        
def shake_robot(robot, shake_delta):
    record = False
    
    if record:
            robot.start_bag_recording()
    joint_degrees = rad2deg(robot.get_joints())
    print('joints:', joint_degrees[5])
    
    if record:
            robot.stop_bag_recording()
    # print('record', record)
    
    joint_degrees[5] +=  shake_delta
    # print('joint_degrees', joint_degrees)
    joint_positions = deg2rad(joint_degrees)
    print('desired rads', joint_positions)
    print('current rads', robot.get_joints())
    
    # use ik_fast to get the best solution
    ik_joints = ik_algorithm.ik_fast(joint_positions)
    print('ik_joints:\n', ik_joints)
    # robot.publisher(joint_positions)
    robot.move_joints(ik_joints , duration=0.1)
    
    # rospy.sleep(3)  # Sleep for 0.5 seconds between movements
    joint_degrees = rad2deg(robot.get_joints())
    print('after',joint_degrees[5])
    
# Define a function to shake both left and right robots
def shake_both_robots(robot_left, robot_right, shake_delta):
    thread_left = threading.Thread(target=shake_robot, args=(robot_left, shake_delta))
    thread_right = threading.Thread(target=shake_robot, args=(robot_right, shake_delta))
    
    # Start both threads
    thread_left.start()
    thread_right.start()
    
    # Wait for both threads to finish
    thread_left.join()
    thread_right.join()



if __name__ == '__main__':
    right_topic = '/right/scaled_pos_joint_traj_controller/state'
    right_command = '/right/scaled_pos_joint_traj_controller/command'
    
    left_topic = '/left/scaled_pos_joint_traj_controller/state'
    left_command = '/left/scaled_pos_joint_traj_controller/command'

    single_topic = '/scaled_pos_joint_traj_controller/state'
    single_command = '/scaled_pos_joint_traj_controller/command'
    try:
        robot_left = DualRobot(left_topic, left_command)
        robot_right = DualRobot(right_topic, right_command)
        
        rospy.init_node('ur5_shake_test', anonymous=True)

        # single robot test
        single_robot = DualRobot(single_topic, single_command)
        shake_delta = 5
        # shake_robot(single_robot, shake_delta)
        
        # shake_robot(robot_right, shake_delta)
        
        
        #  Shake both robots simultaneously
        shake_both_robots(robot_left, robot_right, shake_delta)
        
    except rospy.ROSInterruptException:
        pass