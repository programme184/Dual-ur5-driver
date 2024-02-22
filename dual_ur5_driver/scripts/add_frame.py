#!/usr/bin/env python  
import roslib
roslib.load_manifest('ur_control')
from tf.transformations import quaternion_from_euler
import numpy as np
import rospy
import tf

if __name__ == '__main__':
    degrees = 135
    radians = degrees * (np.pi / 180)
    # yaw_degree = -180
    # yaw_radians = yaw_degree * (np.pi / 180)
    q = quaternion_from_euler(radians, 0, 0)
    q2 = quaternion_from_euler(-radians, 0, 0)
    
    rospy.init_node('fixed_tf_broadcaster')
    
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.25, 0.0),
                         (q[0], q[1], q[2], q[3]),
                         rospy.Time.now(),
                         "dummy",
                         "base_link")
        
        br.sendTransform((0, -0.25, 0.0),
                         (q2[0], q2[1], q2[2], q2[3]),
                         rospy.Time.now(),
                         "test",
                         "base_link")
        rate.sleep()


