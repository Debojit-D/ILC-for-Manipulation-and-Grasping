#!/usr/bin/env python
import rospy
import numpy as np
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations

def minimum_jerk(start, end, duration, num_points):
    """Generates a minimum jerk trajectory from start to end."""
    t = np.linspace(0, duration, num_points)
    traj = start + (end - start) * (10 * (t / duration)**3 - 15 * (t / duration)**4 + 6 * (t / duration)**5)
    return traj

def generate_poses(start_x, end_x, num_points, duration):
    """Generates a list of Point objects based on a minimum jerk trajectory for y-coordinate."""
    x_traj = minimum_jerk(start_x, end_x, duration, num_points)
    return [Point(x=x, y=0.0, z=0.14) for x in x_traj]

def send_pose(poses, path_time):
    rospy.init_node('pose_control_node')
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
    
    roll, pitch, yaw = 0, 0.75, 0  # Replace these with actual values if needed

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    for k, point in enumerate(poses):
        pose = Pose()
        pose.position = point
        pose.orientation = Quaternion(*quaternion)  # Using the converted quaternion
        
        try:
            kinematics_pose = SetKinematicsPoseRequest()
            kinematics_pose.kinematics_pose.pose = pose
            kinematics_pose.planning_group = "manipulator"
            kinematics_pose.end_effector_name = "gripper"
            kinematics_pose.path_time = path_time
            
            response = set_pose(kinematics_pose)
            
            if response.is_planned:
                rospy.loginfo("Pose successfully sent to the robot at index %d.", k)
            else:
                rospy.loginfo("Failed tcleo send pose at index %d.", k)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rospy.sleep(path_time)  # Sleep to respect the path_time for each motion

if __name__ == "__main__":
    # Parameters for the trajectory
    start_x = 0.25
    end_x = 0.05
    duration = 10  # Total duration of the trajectory in seconds
    num_points = 1000  # Number of points in the trajectory
    path_time = 0.0005  # Time allocated for each motion step

    poses = generate_poses(start_x, end_x, num_points, duration)
    send_pose(poses, path_time)
