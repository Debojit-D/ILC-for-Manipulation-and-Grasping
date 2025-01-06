#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def callback(data):
    # Ensure there are efforts (torques) available
    if len(data.effort) == len(data.name):
        for name, torque in zip(data.name, data.effort):
            rospy.loginfo("Joint: %s, Torque: %f", name, torque)
    else:
        rospy.loginfo("Torque data not available for all joints.")

def listener():
    rospy.init_node('joint_torque_reader', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
