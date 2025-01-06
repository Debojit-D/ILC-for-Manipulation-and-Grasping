#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tf_tr

def euler_to_quaternion(roll, pitch, yaw):
    # Convert Euler angles (roll, pitch, yaw) into quaternion
    quaternion = tf_tr.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*quaternion)

def send_pose(poses_with_orientations):
    rospy.init_node('pose_control_node')
    
    # Wait for the service to become available
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
    
    # Main loop for setting poses
    for pose_info in poses_with_orientations:
        position, orientation = pose_info
        pose = Pose()
        pose.position = Point(*position)
        pose.orientation = euler_to_quaternion(*orientation)
        
        try:
            kinematics_pose = SetKinematicsPoseRequest()
            kinematics_pose.kinematics_pose.pose = pose
            kinematics_pose.planning_group = "manipulator"
            kinematics_pose.end_effector_name = "gripper"
            kinematics_pose.path_time = 2  # Define how long the path should take
            
            response = set_pose(kinematics_pose)
            
            if response.is_planned:
                rospy.loginfo("Pose successfully sent to the robot.")
            else:
                rospy.loginfo("Failed to send pose.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rospy.sleep(2)  # Sleep to maintain loop rate

if __name__ == "__main__":
    poses_with_orientations = [
        ((0.0193, 0.1328, 0.001), (0.0, 1.3169, 1.3155)),  # Position (x, y, z), Orientation (Rx, Ry, Rz)
        ((0.167, -0.092, 0.075), (0.0, 1.40, -0.541)),  # Adding an orientation of 45 degrees around y-axis
        ((0.0193, 0.1328, 0.0401), (0.0, 1.3169, 1.3155)),  # Adding an orientation of 90 degrees around z-axis
    ]
    send_pose(poses_with_orientations)
