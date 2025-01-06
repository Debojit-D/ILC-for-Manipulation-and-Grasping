#!/usr/bin/env python
import rospy
import numpy as np
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations

# Minimum jerk trajectory parameters
def minimum_jerk(start_x, end_x, start_z, end_z, num_points, duration):
    """
    Generates a minimum jerk trajectory for both x and z coordinates from start to end.
    """
    T = duration
    t = np.linspace(0, T, num_points)
    x_traj = start_x + (end_x - start_x) * (10 * (t / T)**3 - 15 * (t / T)**4 + 6 * (t / T)**5)
    z_traj = start_z + (end_z - start_z) * (10 * (t / T)**3 - 15 * (t / T)**4 + 6 * (t / T)**5)
    return x_traj, z_traj

def generate_poses(start_x, end_x, start_z, end_z, num_points, y_coordinate, duration):
    """
    Generates a list of Point objects based on a minimum jerk trajectory for x and y coordinates,
    keeping the z-coordinate constant.
    """
    x_traj, z_traj = minimum_jerk(start_x, end_x, start_z, end_z, num_points, duration)
    return [Point(x=x, y=y_coordinate, z=z) for x, z in zip(x_traj, z_traj)]

def send_pose(poses, path_time):
    rospy.init_node('pose_control_node')
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
    
    roll, pitch, yaw = 0, 0.8498, 0  # Replace these with actual values if needed

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
                #rospy.loginfo("Pose successfully sent to the robot at index %d.", k)
                pass
            else:
                rospy.loginfo("Failed to send pose at index %d.", k)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rospy.sleep(path_time)  # Sleep to respect the path_time for each motion

if __name__ == "__main__":
    # Parameters for the trajectory
    start_x = 0.1096
    end_x = 0.1896
    start_z = 0.1358
    end_z = 0.1858
    y_coordinate = 0.00
    duration = 10  # Total duration of the trajectory in seconds
    num_points = 1000  # Number of points in the trajectory
    path_time = 0.01  # Time allocated for each motion step, increased for practical use

    poses = generate_poses(start_x, end_x, start_z, end_z, num_points, y_coordinate, duration)
    send_pose(poses, path_time)
