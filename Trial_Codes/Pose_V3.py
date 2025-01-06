#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion

def send_pose(poses):
    rospy.init_node('pose_control_node')
    
    # Wait for the service to become available
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
    
    # Main loop for setting poses
    for k, point in enumerate(poses):

        pose = Pose()
        pose.position = point
        pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
        
        try:
            kinematics_pose = SetKinematicsPoseRequest()
            kinematics_pose.kinematics_pose.pose = pose
            kinematics_pose.planning_group = "manipulator"
            kinematics_pose.end_effector_name = "gripper"
            kinematics_pose.path_time = 2  # Define how long the path should take
            
            response = set_pose(kinematics_pose)
            
            if response.is_planned:
                rospy.loginfo("Pose successfully sent to the robot at index %d.", k)
            else:
                rospy.loginfo("Failed to send pose at index %d.", k)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rospy.sleep(0.05)  # Sleep to maintain loop rate

if __name__ == "__main__":
    poses = [
        Point(x=0.2, y=-0.20, z=0.2),
        Point(x=0.2, y=+0.18, z=0.2),
        Point(x=0.2, y=-0.20, z=0.2),
    ]
    send_pose(poses)
