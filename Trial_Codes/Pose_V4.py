#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion

def generate_poses(start, end, step):
    """Generates a list of Point objects from start to end using the specified step size."""
    return [Point(x=0.2, y=y, z=0.2) for y in np.arange(start, end + step, step)]

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
            kinematics_pose.path_time = 0.001  # Define how long the path should take
            
            response = set_pose(kinematics_pose)
            
            if response.is_planned:
                rospy.loginfo("Pose successfully sent to the robot at index %d.", k)
            else:
                rospy.loginfo("Failed to send pose at index %d.", k)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rospy.sleep(0.0005)  # Sleep to maintain loop rate

if __name__ == "__main__":
    import numpy as np  # Import numpy for generating ranges

    # Define the start, end, and step size for the y-coordinate
    start = -0.20
    end = 0.20
    step = 0.0005
    
    poses = generate_poses(start, end, step)
    send_pose(poses)
