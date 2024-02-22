#!/usr/bin/env python3
import rospy
import rosbag
import datetime
def sec2time(s):
    return rospy.Time.from_sec(s)

class MyBagReader:
    def __init__(self, bag_file):
        self.bag_file = bag_file
        try:
            self.bag = rosbag.Bag(bag_file, 'r')
        except rosbag.bag.BagException as e:
            rospy.logerr(f"Error opening ROS bag file '{bag_file}': {e}")
            self.bag = None

    def info(self):
        if self.bag is None:
            return

        # Get the list of topics in the bag
        topics = self.bag.get_type_and_topic_info().topics

        # Print information about each topic
        for topic_name, topic_info in topics.items():
            message_type = topic_info.msg_type
            num_messages = topic_info.message_count
            start_time = self.bag.get_start_time()
            end_time = self.bag.get_end_time()

            print(f"Topic: {topic_name}")
            print(f"Message Type: {message_type}")
            print(f"Number of Messages: {num_messages}")
            print(f"Start Time: {start_time}")
            print(f"End Time: {end_time}")
            print("--------------------")

        # Read and process messages from the bag
        for topic, msg, timestamp in self.bag.read_messages():
            # You can access and process the message data here as needed
            # For example, you can print the message data:
            print(f"Received message on topic '{topic}' at time {timestamp}:")
            print(msg)
            print("--------------------")
            break

    def sample_attribute(self, topic, time_interval):
        ret = []
        try:
            # Get the start and end times of the bag
            start_time = self.bag.get_start_time()
            end_time = self.bag.get_end_time()

            # Initialize the current time
            current_time = start_time

            # Iterate from start to end with the specified time interval
            while current_time <= end_time:
                # Read the message at the current time
                for read_topic, msg, timestamp in self.bag.read_messages(topics=None, start_time=sec2time(current_time), end_time=sec2time(current_time + time_interval)):
                    # Process the message data as needed
                    if read_topic==topic:
                        # Extract and print the position
                        data = msg.actual.positions
                        ret.append(data)
                    break
                # Increment the current time by the time interval
                current_time += time_interval
            
            return ret
        except Exception as ex:
            rospy.logerr(f"An error occurred while reading ROS bag file '{self.bag_file}': {ex}")


if __name__ == '__main__':
    try:
        rospy.init_node('read_rosbag_at_interval', anonymous=True)

        # Create a MyBagReader object
        import sys
        bag_reader = MyBagReader(sys.argv[1])

        # Specify the time interval for sampling (0.1 seconds in this example)
        print(bag_reader.info())
        data = bag_reader.sample_attribute(topic='/scaled_pos_joint_traj_controller/state', time_interval=0.1)
        print(data)
        
    except rospy.ROSInterruptException:
        pass
