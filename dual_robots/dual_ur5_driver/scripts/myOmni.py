#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniButtonEvent
from rosbag import Bag  # Import the rosbag library
import time

class MyBag:
    def __init__(self, topic, filename):
        self.bag = None  # Initialize a bag object
        self.recording_flag = False
        self.filename = filename
        self.topic = topic  # Set the topic for recording

    def start_bag_recording(self):
        self.recording_flag = True
        # Generate a bag file name based on the current timestamp
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        bag_file_name = f"bags/{self.filename}@{timestamp}.bag"
        
        # Start recording data to the bag file
        self.bag = Bag(bag_file_name, 'w')  # Open the bag file for writing

    def stop_bag_recording(self):
        self.recording_flag = False
        # Stop recording and close the bag file
        if self.bag is not None:
            self.bag.close()
            rospy.loginfo("Bag recording stopped and saved.")
            self.bag = None
            
    def subscriber_callback(self, data):
        if self.recording_flag and self.bag is not None:
            self.bag.write(self.topic, data)

class MyOmni(MyBag):
    def __init__(self):
        topic = '/phantom/phantom/joint_states'  # Define the topic for Omni joint states
        super().__init__(topic, filename="omni")  # Initialize the MyBag base class with the topic
        self.gray_button = 0
        self.white_button = 0
        self.gray_button_flag = False 
        self.white_button_flag = False

        # Create subscribers for the button and joint states
        rospy.Subscriber(topic, JointState, self.subscriber_callback)
        rospy.Subscriber('/phantom/phantom/button', OmniButtonEvent, self.button_event_callback)

        rospy.sleep(0.5)

    def button_event_callback(self, data):
        # Process the button event
        self.gray_button = data.grey_button
        self.white_button = data.white_button

        if data.grey_button > 0:
            rospy.loginfo("Gray Button pressed!")
            self.gray_button_flag = not self.gray_button_flag  # Toggle the gray button flag
        
        if data.white_button > 0:
            rospy.loginfo("White Button pressed!")
            self.white_button_flag = not self.white_button_flag

    def subscriber_callback(self, data):
        super().subscriber_callback(data) 
        self.joint_positions = np.array(data.position)

    def get_joints(self):
        return np.array(self.joint_positions)

if __name__ == '__main__':
    try:
        rospy.init_node('phantom_omni_joint_echo')
        my_omni = MyOmni()  # Initialize the myOmni object
        prev_flag = False  # Initialize the previous button flag
        while not rospy.is_shutdown():
            # Check if the gray button state has changed and recording is not active
            if my_omni.gray_button and not my_omni.recording_flag:
                my_omni.start_bag_recording()
            # Check if the gray button state has changed and recording is active
            elif not my_omni.gray_button and my_omni.recording_flag:
                my_omni.stop_bag_recording()

            # Print joint states while recording is active
            if my_omni.recording_flag:
                omni_joints = my_omni.get_joints()
                print("Omni Joints:", omni_joints)

            rospy.sleep(0.1)  # Control the loop rate

    except rospy.ROSInterruptException:
        pass
