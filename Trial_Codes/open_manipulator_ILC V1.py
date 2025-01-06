#!/usr/bin/env python
import rospy
import numpy as np
import pandas as pd
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from tf.transformations import quaternion_from_euler
import time
from geometry_msgs.msg import Point, Vector3
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations 

################################################################## INITIALIZING THE ROBOT #######################################################################

def send_pose_home():
    rospy.wait_for_service('/goal_task_space_path')
    try:
        set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
        
        # Define the pose
        target_pose = Pose()
        target_pose.position = Point(x=0.05, y=-0.0, z=0.13)
        roll, pitch, yaw = 0, 0.75, 0
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        target_pose.orientation = Quaternion(*quaternion)
        
        # Define the kinematics pose
        kinematics_pose = SetKinematicsPoseRequest()
        kinematics_pose.kinematics_pose.pose = target_pose
        kinematics_pose.planning_group = "manipulator"
        kinematics_pose.end_effector_name = "gripper"
        kinematics_pose.path_time = 1

        # Call the service
        response = set_pose(kinematics_pose)
        if response.is_planned:
            rospy.loginfo("Pose successfully sent to the robot.")
        else:
            rospy.loginfo("Failed to send pose.")
            
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

################################################################## FUNCTION TO PUBLISH THE POSES #######################################################################

def send_pose(poses, path_time):
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
    
    roll, pitch, yaw = 0, 0.75, 0
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    for k, point in enumerate(poses):
        pose = Pose()
        pose.position = point
        pose.orientation = Quaternion(*quaternion)
        
        try:
            kinematics_pose = SetKinematicsPoseRequest()
            kinematics_pose.kinematics_pose.pose = pose
            kinematics_pose.planning_group = "manipulator"
            kinematics_pose.end_effector_name = "gripper"
            kinematics_pose.path_time = path_time
            
            response = set_pose(kinematics_pose)
            if response.is_planned:
                pass
            else:
                rospy.loginfo("Failed to send pose at index %d.", k)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rospy.sleep(path_time)

################################################################## INITIALIZATIONS #######################################################################
# Controller Properties
dt = 1/100.0  # Time step
T = 2  # Total time for each trajectory
t = np.arange(0, T, dt)
n_samples = int(T/dt)  # Number of samples in the trajectory
freq = 1/dt  # Frequency of control loop

# Kinematics and trajectory
start_x = 0.25  # Starting x-coordinate
end_x = 0.05  # Ending x-coordinate
start_y = 0.00
end_y = 0.00
z_coordinate = 0.14  # Constant z-coordinate

# System parameters for the controller
s = 10
b = 1
Ac = np.array([[-s/b, 0], [0, -s/b]])  # Continuous-time system matrix (example)
A = np.eye(2) + Ac * dt  # Discrete-time system matrix
Bc = np.array([[1/b, 0], [0, 1/b]])  # Continuous-time input matrix
B = Bc * dt  # Discrete-time input matrix
C = np.eye(2)  # Output matrix

# Initialize state and input vectors
X = np.zeros((2, n_samples))  # System states
U = np.zeros((2, n_samples))  # Control inputs

# ILC specific parameters
u_ilc = np.zeros((2, n_samples))  # ILC input updates
u_ff = np.zeros((2, n_samples))  # Feedforward inputs

# Error tracking for learning
err = np.zeros(n_samples)  # Error per sample

# Constants for the control logic
BL = 20  # Baseline period
PT = 0  # Perturbation period
WS = 0   # Washout period
RP = 0   # Recovery period
trials = BL + PT + WS + RP

# Minimum jerk trajectory parameters
def minimum_jerk(start_x, end_x, start_y, end_y, num_points):
    t = np.linspace(0, T, num_points)
    x_traj = start_x + (end_x - start_x) * (10 * (t / T)**3 - 15 * (t / T)**4 + 6 * (t / T)**5)
    y_traj = start_y + (end_y - start_y) * (10 * (t / T)**3 - 15 * (t / T)**4 + 6 * (t / T)**5)
    return x_traj, y_traj

def generate_poses(start_x, end_x, start_y, end_y, num_points, z_coordinate):
    x_traj, y_traj = minimum_jerk(start_x, end_x, start_y, end_y, num_points)
    return [Point(x=x, y=y, z=z_coordinate) for x, y in zip(x_traj, y_traj)]

# Controller function placeholders
def sat_u(v, saturation_min=-50, saturation_max=50):
    return np.clip(v, saturation_min, saturation_max)

# Trajectory and control initialization
xd, yd = minimum_jerk(start_x, end_x, start_y, end_y, n_samples)  # Desired trajectories in x and y
zd = np.ones(n_samples) * z_coordinate  # Desired trajectory in z (constant)

################################################################## ESSENTIAL FUNCTIONS #######################################################################

# Global data storage for synchronization
latest_data = {
    'torques': None,
    'jacobian': None,
    'g_matrix': None,
    'desired_trajectory': np.vstack((xd, yd)),  # Initialize with the desired trajectory
    'external_forces': np.zeros((2, n_samples))  # Initialize with zero external forces
}

def joint_torque_callback(data):
    torques = np.array(data.data[:-1])
    update_data_storage('torques', torques)

def jacobian_callback(data):
    jacobian = np.array(data.data).reshape((6, 4))  # Adjust the shape as needed
    update_data_storage('jacobian', jacobian)

def g_matrix_callback(data):
    g_matrix = np.array(data.data)
    update_data_storage('g_matrix', g_matrix)

def update_data_storage(data_type, data):
    latest_data[data_type] = data
    # Check if all elements in latest_data are not None
    if all(value is not None for value in latest_data.values()):
        compute_ilc_control()

def compute_ilc_control():
    jacobian_transpose = np.transpose(latest_data['jacobian'])
    jacobian_inverse_transpose = np.linalg.pinv(jacobian_transpose)
    net_joint_torques = latest_data['torques'] - latest_data['g_matrix']
    end_effector_forces = np.dot(jacobian_inverse_transpose, net_joint_torques)
    #print("End Effector Forces:", end_effector_forces)

def controller(XD, X, u_ilc, U, t, n, Fext):
    X[0, 0] = 0
    X[1, 0] = 0

    for k in range(0, len(t) - 1):
        X[:, k + 1] = np.matmul(A, X[:, k]) + np.matmul(B, U[:, k] + Fext[:, k])
    XM = np.vstack((X[0, :], X[1, :]))
    TE = (XD - XM)
    u_ilc = 0.9999 * u_ilc + 0.9 * np.tanh(TE)
    u_ilc = sat_u(u_ilc)
    U = (1 * u_ilc)
    return X, u_ilc, U, TE

def trial_loop():
    global X, u_ilc, U, latest_data
    for n in range(trials):
        X, u_ilc, U, TE = controller(latest_data['desired_trajectory'], X, u_ilc, U, t, n, latest_data['external_forces'])
        poses = generate_poses(X[0, 0], X[0, -1], X[1, 0], X[1, -1], n_samples, z_coordinate)
        send_pose(poses, dt)
        print(f"Trial {n}, Error: {TE}")
    rospy.sleep(0.5)
    send_pose_home()

def main():
    rospy.init_node('ilc_controller', anonymous=True)
    rospy.Subscriber('/joint_torques', Float64MultiArray, joint_torque_callback)
    rospy.Subscriber('/jacobian_values', Float64MultiArray, jacobian_callback)
    rospy.Subscriber('/g_matrix_values', Float64MultiArray, g_matrix_callback)
    rospy.spin()

if __name__ == "__main__":
    send_pose_home()
    main()