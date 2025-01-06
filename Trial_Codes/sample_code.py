#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math

def joint_state_publisher():
    rospy.init_node('joint_state_publisher_node', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        joint_state = JointState()
        
        # Populate header
        joint_state.header.stamp = rospy.Time.now()

        # Define joint names
        joint_state.name = ["joint1", "joint2", "joint3", "joint4"]

        # Define joint positions (angles in radians)
        joint_state.position = [
            1.54,  # Example angle for joint1
            math.cos(rospy.Time.now().to_sec()),  # Example angle for joint2
            math.sin(rospy.Time.now().to_sec() * 2),  # Example angle for joint3
            math.cos(rospy.Time.now().to_sec() * 2)   # Example angle for joint4
        ]

        # Define joint velocities and efforts if needed
        joint_state.velocity = [0.1, 0.1, 0.1, 0.1]
        joint_state.effort = []

        # Publish the joint states
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass
