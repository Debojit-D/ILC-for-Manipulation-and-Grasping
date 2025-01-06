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
        
        target_pose = Pose()
        target_pose.position = Point(x=0.05, y=0.0, z=0.16)
        roll, pitch, yaw = 0, 0.90, 0
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        target_pose.orientation = Quaternion(*quaternion)
        
        kinematics_pose = SetKinematicsPoseRequest()
        kinematics_pose.kinematics_pose.pose = target_pose
        kinematics_pose.planning_group = "manipulator"
        kinematics_pose.end_effector_name = "gripper"
        kinematics_pose.path_time = 2

        response = set_pose(kinematics_pose)
        if response.is_planned:
            #rospy.loginfo(" Home pose successfully sent to the robot.")
            pass
        else:
            rospy.loginfo("Failed to send pose.")
            
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

################################################################## FUNCTION TO PUBLISH THE POSES #######################################################################

def send_pose(poses, path_time):
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
    
    roll, pitch, yaw = 0, 0.90, 0
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
                #rospy.loginfo("Pose successfully sent to the robot.")
                pass
            else:
                rospy.loginfo("Failed to send pose at index.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rospy.sleep(path_time)

################################################################## INITIALIZATIONS #######################################################################
# Controller Properties
dt = 1/100.0  # Time step
T = 1 # Total time for each trajectory
t = np.arange(0, T, dt)
n_samples = int(T/dt)  # Number of samples in the trajectory
freq = 1/dt  # Frequency of control loop

# Kinematics and trajectory
start_x = 0.05  # Starting x-coordinate
end_x = 0.13  # Ending x-coordinate
start_y = 0.00
end_y = 0.00
z_coordinate = 0.16  # Constant z-coordinate

# System parameters for the controller
s = 1
b = 10
Ac = np.array([[-s/b, 0], [0, -s/b]])  # Continuous-time system matrix (example) 
A = np.eye(2) + Ac * dt  # Discrete-time system matrix
Bc = np.array([[1/b, 0], [0, 1/b]])  # Continuous-time input matrix D
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
BL = 50  # Baseline period
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

# Trajectory and control initialization
xd, yd = minimum_jerk(start_x, end_x, start_y, end_y, n_samples)  # Desired trajectories in x and y
#print(xd)
#print(yd)
xd = np.ones(n_samples) * xd  # Desired trajectory in x (constant)
yd = np.ones(n_samples) * yd  # Desired trajectory in y (constant)
zd = np.ones(n_samples) * z_coordinate  # Desired trajectory in z (constant)
XD = np.vstack((xd,yd))
end_effector_positions = np.array([[start_x] * n_samples, [start_y] * n_samples])
end_effector_forces_trial_array = np.zeros((2, n_samples))  # Initialize with zero external forces



################################################################## ESSENTIAL FUNCTIONS #######################################################################

# Controller function placeholders
def sat_u(v, saturation_min=-50, saturation_max=50):
    return np.clip(v, saturation_min, saturation_max)

# ESSENTIAL FUNCTIONS

# Global data storage for synchronization
latest_data = {
    'torques': None,
    'jacobian': None,
    'g_matrix': None,
    'desired_trajectory': np.vstack((xd, yd)),  # Initialize with the desired trajectory
    'external_forces': np.zeros((2, n_samples))  # Initialize with zero external forces
}

def joint_torque_callback(data):
    torques = np.array(data.data)[:-1]  # Drop the last value (for the gripper)
    update_data_storage('torques', torques)

def joint_state_callback(data):
    if len(data.effort) >= 5:  # Ensuring there are enough entries for all joints
        currents = np.array(data.effort[:4])  # Assuming the last value is for the gripper and we don't need it
        # Define the torque constants for each joint, these should be obtained from the datasheet
        torque_constants = np.array([1.793209/1000, 1.793209/1000, 1.793209/10000, 1.793209/1000])  # Replace K_t1, ..., K_t4 with actual values
        torques = torque_constants * currents  # Element-wise multiplication
        update_data_storage('torques', torques)
    else:
        rospy.logwarn("Received joint state message with insufficient length of effort array")

def jacobian_callback(data):
    jacobian = np.array(data.data).reshape((6, 4))  # Adjust the shape as needed
    update_data_storage('jacobian', jacobian)

def g_matrix_callback(data):
    g_matrix = np.array(data.data)
    update_data_storage('g_matrix', g_matrix)

def end_effector_callback(data):
    end_effector_position = np.array([data.x, data.y])
    update_data_storage('end_effector_position', end_effector_position)

def update_data_storage(data_type, data):
    latest_data[data_type] = data

def compute_end_effector_forces(k):
    """ Compute end-effector forces for the current timestep k and update latest_data for this timestep. """
    if all(value is not None for value in [latest_data['torques'], latest_data['jacobian'], latest_data['g_matrix']]):
        jacobian_transpose = np.transpose(latest_data['jacobian'])
        jacobian_inverse_transpose = np.linalg.pinv(jacobian_transpose)
        net_joint_torques = latest_data['torques'][:4] - latest_data['g_matrix']
        end_effector_forces = np.dot(jacobian_inverse_transpose, net_joint_torques)
        latest_data['external_forces'] = end_effector_forces[:2]

# def force_wrapper(force_measured, f_previous):
#     x=force_measured
#     f_hat= np.abs(np.tanh((x**28)*np.exp(30*np.abs(x))))*x
#     f = f_previous + 0.8*(f_hat - f_previous)
#     return f

def force_wrapper(force_measured, f_previous):
    x=force_measured
    f_hat= (np.tanh(x**8))*x
    f = f_previous + 0.8*(f_hat - f_previous)
    return f

def controller(XD, X, u_ilc, U):
    #print("Shape of X:", X.shape)
    #print("Shape of U:", U.shape)
    #print("Shape of Fext:", Fext.shape)
    #print(Fext)

    
    TE = (XD - X)
    #print(TE[0,-1])
    print(TE)
    u_ilc = 0.96 * u_ilc + 8 * TE
    u_ilc = sat_u(u_ilc)
    U = (1 * u_ilc)
    return X, u_ilc, U

X= np.zeros((2, n_samples))
u_ilc = np.zeros((2, n_samples))
U = np.zeros((2, n_samples))
f_previous = np.zeros(2)
end_effector_positions = np.array([[start_x] * n_samples, [start_y] * n_samples])


def trial_loop():
    
    global X, u_ilc, U, latest_data, end_effector_forces_trial_array, f_previous
    
    for n in range(trials):

        X[0,0] = start_x
        X[1,0] = start_y


        
        X, u_ilc, U = controller(XD, end_effector_positions, u_ilc, U)
    

        for k in range(0, n_samples-1):
            # X[:, k + 1] = np.matmul(A, X[:, k]) + np.matmul(B, (U[:, k] + Fext[:, k]))
            compute_end_effector_forces(k)
            eff = latest_data['external_forces']
            print(eff)
            eff = force_wrapper(eff, f_previous)
            
            f_previous = eff
            eff_expanded = eff.reshape(2, 1)
            force_input = np.matmul(B, (-eff_expanded + U[:, k:k+1]))
            #force_input = np.matmul(B, (-eff_expanded ))
            force_input = np.squeeze(force_input)  # Ensure it is (2,)

            X[:, k + 1] = np.matmul(A, X[:, k]) + force_input
            

            pose = Point(x=X[0, k+1], y=X[1, k+1], z=z_coordinate)
            send_pose([pose], dt)
            # #Store the current end-effector position
            if 'end_effector_position' in latest_data and latest_data['end_effector_position'] is not None:
                end_effector_positions[:, k] = latest_data['end_effector_position']
            else:
                end_effector_positions[:, k] = [0.0, 0.0]  # Fallback
            rospy.sleep(dt)
                

        rospy.sleep(3)
        send_pose_home()
        rospy.loginfo("Trial loop %d completed.", n + 1)
        rospy.sleep(2)  # Rest for 2 seconds after each trial loop
        

def all_data_received():
    return all(value is not None for key, value in latest_data.items() if key not in ['desired_trajectory', 'external_forces'])

def wait_for_data():
    rospy.loginfo("Waiting for all data to be received...")
    rate = rospy.Rate(100)  # Check at 10 Hz
    while not all_data_received():
        rate.sleep()
    rospy.loginfo("All data received. Starting the trial loop.")

def main():
    rospy.init_node('ilc_controller', anonymous=True)
    #rospy.Subscriber('/joint_torques', Float64MultiArray, joint_torque_callback) was causing Buffer Overrun Issues 
    rospy.Subscriber('/jacobian_values', Float64MultiArray, jacobian_callback)
    rospy.Subscriber('/g_matrix_values', Float64MultiArray, g_matrix_callback)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.Subscriber('/end_effector_position', Point, end_effector_callback)

    
    send_pose_home()
    rospy.sleep(2)
    wait_for_data()  # Wait for all data to be received
    trial_loop()
    rospy.spin()

if __name__ == "__main__":
    main()