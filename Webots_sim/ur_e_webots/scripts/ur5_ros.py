#!/usr/bin/env python3

"""universal_robot_ros controller."""

import argparse
import rospy

from joint_state_publisher import JointStatePublisher
from controller import Robot
from trajectory_follower import TrajectoryFollower
from rosgraph_msgs.msg import Clock


parser = argparse.ArgumentParser()
parser.add_argument('--node-name', dest='nodeName', default='ur_driver', help='Specifies the name of the node.')
arguments, unknown = parser.parse_known_args()

rospy.init_node(arguments.nodeName, disable_signals=True)

jointPrefix = rospy.get_param('prefix', '')
if jointPrefix:
    print('Setting prefix to %s' % jointPrefix)

robot = Robot()
nodeName = arguments.nodeName + '/' if arguments.nodeName != 'ur_driver' else ''
jointStatePublisher = JointStatePublisher(robot, jointPrefix, nodeName)
trajectoryFollower = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, nodeName)
trajectoryFollower.start()

# we want to use simulation time for ROS
clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
if not rospy.get_param('use_sim_time', False):
    rospy.logwarn('use_sim_time is not set!')

timestep = int(robot.getBasicTimeStep())

echo = 0
while robot.step(timestep) != -1 and not rospy.is_shutdown():
    jointStatePublisher.publish()
    trajectoryFollower.update()
    # pulish simulation clock
    msg = Clock()
    time = robot.getTime()
    msg.clock.secs = int(time)
    
    # round prevents precision issues that can cause problems with ROS timers
    # msg.clock.nsecs = round(1000 * (time - msg.clock.secs)) * 1.0e+6
    
    if echo < 2:
        print('time:\n', time)
        print('msg:\n', msg, '\n', msg.clock.nsecs)
    echo +=1
    clockPublisher.publish(msg)
