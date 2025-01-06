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

################################################################## INITALIZING THE ROBOT #######################################################################

def send_pose_home():
    rospy.init_node('set_kinematics_pose_client')
    
    rospy.wait_for_service('/goal_task_space_path')
    try:
        set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
        
        # Define the pose
        target_pose = Pose()
        target_pose.position = Point(x=0.05  , y=-0.0, z=0.13)
        # Define the Euler angles (roll, pitch, yaw)
        roll = 0  # in radians
        pitch = 0.75  # in radians
        yaw = 0  # in radians

        # Convert the Euler angles to a quaternion
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        # Set the pose orientation using the converted quaternion
        target_pose.orientation = Quaternion(*quaternion)
        
        # Define the kinematics pose
        kinematics_pose = SetKinematicsPoseRequest()
        kinematics_pose.kinematics_pose.pose = target_pose
        kinematics_pose.planning_group = "manipulator"
        kinematics_pose.end_effector_name = "gripper"
        kinematics_pose.path_time = 1
        #kinematics_pose.max_accelerations_scaling_factor = 0.0
        #kinematics_pose.max_velocity_scaling_factor = 0.0
        #kinematics_pose.tolerance = 0.0

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
                #rospy.loginfo("Pose successfully sent to the robot at index %d.", k)
                pass
            else:
                rospy.loginfo("Failed to send pose at index %d.", k)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rospy.sleep(dt) 

##################################################################   INITIALIZATIONS   #######################################################################
# Controller Properties
dt = 1/100.0  # Time step
T = 2  # Total time for each trajectory
t = np.arange
n_samples = int(T/dt)  # Number of samples in the trajectory
freq = 1/dt  # Frequency of control loop

# Kinematics and trajectory
start_x = 0.25  # Starting x-coordinate
end_x = 0.05  # Ending x-coordinate
start_y = 0.00
end_y = 0.00
z_coordinate = 0.14  # Constant z-coordinate

# System parameters for the controller
s=10
b=1
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
    """
    Generates a minimum jerk trajectory for both x and y coordinates from start to end.
    """
    t = np.linspace(0, T, num_points)
    x_traj = start_x + (end_x - start_x) * (10 * (t / T)**3 - 15 * (t / T)**4 + 6 * (t / T)**5)
    y_traj = start_y + (end_y - start_y) * (10 * (t / T)**3 - 15 * (t / T)**4 + 6 * (t / T)**5)
    return x_traj, y_traj

def generate_poses(start_x, end_x, start_y, end_y, num_points, z_coordinate):
    """
    Generates a list of Point objects based on a minimum jerk trajectory for x and y coordinates,
    keeping the z-coordinate constant.
    """
    x_traj, y_traj = minimum_jerk(start_x, end_x, start_y, end_y, num_points)
    return [Point(x=x, y=y, z=z_coordinate) for x, y in zip(x_traj, y_traj)]

# Controller function placeholders
def sat_u(v, saturation_min=-50, saturation_max=50):
    """Saturates the input v within the given minimum and maximum bounds."""
    return np.clip(v, saturation_min, saturation_max)

# Trajectory and control initialization
xd, yd = minimum_jerk(start_x, end_x, start_y, end_y, n_samples)  # Desired trajectories in x and y
zd = np.ones(n_samples) * z_coordinate  # Desired trajectory in z (constant)

##################################################################   ESSENTIAL FUCNTIONS   #######################################################################

# Global data storage for synchronization
latest_data = {
    'torques': None,
    'jacobian': None,
    'g_matrix': None
}

def joint_torque_callback(data):
    update_data_storage('torques', np.array(data.data))

def jacobian_callback(data):
    jacobian = np.array(data.data).reshape((6, 4))  # Adjust the shape as needed
    update_data_storage('jacobian', jacobian)

def g_matrix_callback(data):
    g_matrix = np.array(data.data)
    update_data_storage('g_matrix', g_matrix)

# Assuming you might still want to fetch and use position and orientation in your ILC calculations
def position_callback(data):
    update_data_storage('position', data)

def orientation_callback(data):
    update_data_storage('orientation', data)


def update_data_storage(data_type, data):
    """ Update the specific type of data in the global storage and try computing forces if all data is ready. """
    latest_data[data_type] = data
    if all(latest_data.values()):  # Check if all data types are non-None
        compute_ilc_control()

def compute_ilc_control():
    """ Compute the ILC control based on the latest available data. """
    # Compute the Jacobian transpose inverse
    jacobian_transpose = np.transpose(latest_data['jacobian'])
    jacobian_inverse_transpose = np.linalg.pinv(jacobian_transpose)
    
    # Calculate net joint torques (subtracting gravity effects)
    net_joint_torques = latest_data['torques'] - latest_data['g_matrix']
    
    # Calculate end-effector forces
    end_effector_forces = np.dot(jacobian_inverse_transpose, net_joint_torques)
    #print("End Effector Forces:", end_effector_forces)

def controller(XD,X,u_ilc,U,t,n,Fext):
    # Initial Conditions
    X[0,0]=0
    X[1,0]=0

    #Twin
    for k in range(0,len(t)-1):
        X[:,k+1]= np.matmul(A,X[:,k])  + np.matmul(B,U[:,k] + Fext[:,k] )
  
    # Errors
    XM=np.vstack((X[0,:],X[1,:]))
    TE=(XD-XM)
    
    #Controller
    u_ilc= 0.9999*u_ilc + 0.9*np.tanh(TE)
    u_ilc=sat_u(u_ilc)
    U=(1*u_ilc)

    return X,u_ilc,U


def trial_loop():
    global X, u_ilc, U, latest_data
    for n in range(trials):
        X, u_ilc, U, TE = controller(latest_data['desired_trajectory'], X, u_ilc, U, t, n, latest_data['external_forces'])
        poses = generate_poses(X[0, 0], X[0, -1], X[1, 0], X[1, -1], n_samples, z_coordinate)
        send_pose(poses, dt)
        #print(f"Trial {n}, Error: {TE}")
    rospy.sleep(0.5)
    send_pose_home()



def main():
    rospy.init_node('ilc_controller', anonymous=True)

    # Subscribe to the necessary topics to receive the robot's state and control information
    rospy.Subscriber('/joint_torques', Float64MultiArray, joint_torque_callback)
    rospy.Subscriber('/jacobian_values', Float64MultiArray, jacobian_callback)
    rospy.Subscriber('/g_matrix_values', Float64MultiArray, g_matrix_callback)

    # Optionally, subscribe to these if you plan to use position and orientation data
    rospy.Subscriber('/end_effector_position', Point, position_callback)
    rospy.Subscriber('/end_effector_orientation', Vector3, orientation_callback)

    # Start the ROS event loop to continuously listen for new messages on subscribed topics
    rospy.spin()

if __name__ == "__main__":
    send_pose_home()
    main()


