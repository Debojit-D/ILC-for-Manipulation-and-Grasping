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
import os
import signal 

################################################################## INITIALIZING THE ROBOT #######################################################################

def send_pose_home():
    rospy.wait_for_service('/goal_task_space_path')
    try:
        set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
        
        target_pose = Pose()
        target_pose.position = Point(x=0.08, y=0.0, z=0.197)
        roll, pitch, yaw = 0, 0.3, 0
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
    
    roll, pitch, yaw = 0, 0.3, 0
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
start_x = 0.08  # Starting x-coordinate
end_x = 0.19  # Ending x-coordinate
start_y = 0.00
end_y = 0.00
z_coordinate = 0.197 # Constant z-coordinate

# System parameters for the controller
s = 1
b = 10
Ac = np.array([[-s/b, 0], [0, -s/b]])  # Continuous-time system matrix (example) 
A = np.eye(2) + Ac * dt  # Discrete-time system matrix
Bc = np.array([[1/b, 0], [0, 1/b]])  # Continuous-time input matrix D
B = Bc * dt  # Discrete-time input matrix
C = np.eye(2)  # Output matrix

# Constants for the control logic
BL = 50  # Baseline period
PT = 0  # Perturbation period
WS = 0   # Washout period
RP = 0   # Recovery period
trials = BL + PT + WS + RP

# Initialize state and input vectors
X = np.zeros((2, n_samples))  # System states
U = np.zeros((2, n_samples))  # Control inputs

# ILC specific parameters
u_ilc = np.zeros((2, n_samples))  # ILC input updates
u_ff = np.zeros((2, n_samples))  # Feedforward inputs

# Error tracking for learning
err = np.zeros(n_samples)  # Error per sample

f_previous = np.zeros(2)
end_effector_positions = np.array([[start_x] * n_samples, [start_y] * n_samples])

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


############################################################### Data Logging and and Update Functions #############################################################
# Global data storage for synchronization
latest_data = {
    'torques': None,
    'jacobian': None,
    'g_matrix': None,
    'desired_trajectory': np.vstack((xd, yd)),  # Initialize with the desired trajectory
    'external_forces': np.zeros((2, n_samples)),  # Initialize with zero external forces
    'joint_angles': None  # Initialize joint angles to ensure the key exists
}

trial_data = {
    'cartesian_positions': {},
    'forces': {},
    'joint_angles': {},
    'torques': {}
}

te_data = []
u_ilc_data = []

def joint_torque_callback(data):
    torques = np.array(data.data)[:-1]  # Drop the last value (for the gripper)
    update_data_storage('torques', torques)

def joint_state_callback(data):
    if len(data.effort) >= 5 and len(data.position) >= 5:  # Ensuring there are enough entries for all joints
        # Process torques
        currents = np.array(data.effort[:4])  # Assuming the last value is for the gripper and we don't need it
        torque_constants = np.array([1.793209/1000, 1.793209/1000, 1.793209/10000, 1.793209/1000])  # Replace with actual values
        torques = torque_constants * currents  # Element-wise multiplication
        update_data_storage('torques', torques)

        # Process joint angles
        joint_angles = np.array(data.position[:4])  # Assuming the first four positions are the needed joint angles
        update_data_storage('joint_angles', joint_angles)

    else:
        if len(data.effort) < 5:
            rospy.logwarn("Received joint state message with insufficient length of effort array")
        if len(data.position) < 5:
            rospy.logwarn("Received joint state message with insufficient length of position array")

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
        latest_data['external_forces'] = 0*end_effector_forces[:2]

################################################################## ESSENTIAL FUNCTIONS USED IN ILC LOOP #######################################################################

# def force_wrapper(force_measured, f_previous):
#     x=force_measured
#     f_hat= np.abs(np.tanh((x**28)*np.exp(30*np.abs(x))))*x
#     f = f_previous + 0.8*(f_hat - f_previous)
#     return f

# def force_wrapper(force_measured, f_previous):
#     x=force_measured
#     f_hat= (np.tanh(x**8))*x
#     f = f_previous + 0.8*(f_hat - f_previous)
#     return f

def force_wrapper(force_measured, f_previous):
    # Isolate components
    x = force_measured[0]  # Take the x-component for modification
    y = force_measured[1]  # Preserve the y-component without modification

    # Apply the transformation only to the x-component
    f_hat_x= (np.tanh(x**8))*x
    
    # Update both x and y components of the force vector based on the previous values
    f_x = f_previous[0] + 0.8 * (f_hat_x - f_previous[0])
    f_y = f_previous[1] + 0.8 * (y - f_previous[1])  # Update y-component based on its own current and previous value

    # Combine the updated x and y components
    f = np.array([f_x, f_y])
    return f

def controller(XD, X, u_ilc, U):
    #print("Shape of X:", X.shape)
    #print("Shape of U:", U.shape)
    #print("Shape of Fext:", Fext.shape)
    #print(Fext)

    TE = (XD - X)
    #print(TE[0,-1])
    #print(TE)
    u_ilc = 0.97 * u_ilc + 8 * TE
    
    te_data.append(TE.tolist())  # Convert numpy array to list
    u_ilc_data.append(u_ilc.tolist())  # Convert numpy array to list
    
    u_ilc = sat_u(u_ilc)
    U = (1 * u_ilc)
    return X, u_ilc, U

# Controller function placeholders
def sat_u(v, saturation_min=-50, saturation_max=50):
    return np.clip(v, saturation_min, saturation_max)

def trial_loop():
    
    global X, u_ilc, U, latest_data, end_effector_forces_trial_array, f_previous
    
    for n in range(trials):

        X[0,0] = start_x
        X[1,0] = start_y

        trial_name = f'TN{n+1}'
        trial_data['cartesian_positions'][trial_name] = []
        trial_data['forces'][trial_name] = []
        trial_data['joint_angles'][trial_name] = []
        trial_data['torques'][trial_name] = [] 

        X, u_ilc, U = controller(XD, end_effector_positions, u_ilc, U)
    

        for k in range(0, n_samples-1):
            # X[:, k + 1] = np.matmul(A, X[:, k]) + np.matmul(B, (U[:, k] + Fext[:, k]))
            compute_end_effector_forces(k)
            eff = latest_data['external_forces']
            print(eff)
            #eff = force_wrapper(eff, f_previous)
            
            f_previous = eff
            eff_expanded = eff.reshape(2, 1)
            force_input = np.matmul(B, (-eff_expanded + U[:, k:k+1]))
            #force_input = np.matmul(B, (-eff_expanded ))
            force_input = np.squeeze(force_input)  # Ensure it is (2,)

            X[:, k + 1] = np.matmul(A, X[:, k]) + force_input
           
            current_time = rospy.get_time()
            trial_data['cartesian_positions'][trial_name].append((current_time, X[0, k], X[1, k]))
            trial_data['forces'][trial_name].append((current_time, latest_data['external_forces'][0], latest_data['external_forces'][1]))
            trial_data['joint_angles'][trial_name].append((current_time, latest_data['joint_angles']))
            trial_data['torques'][trial_name].append((current_time, latest_data['torques']))  # Append torques


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
    
    save_trial_data()
    save_te_data()
    save_u_ilc_data()
        

###################################################################### DATA STORAGE FUNCTIONS ####################################################################
def save_trial_data():
    csv_directory = os.path.join(os.path.expanduser('~'), 'trajectory_data')
    os.makedirs(csv_directory, exist_ok=True)
    rospy.loginfo(f"Data will be saved in: {csv_directory}")

    # Prepare a list to collect all data
    all_data = []
    
    # Collect data from each trial
    for trial_name in trial_data['cartesian_positions'].keys():
        positions = trial_data['cartesian_positions'][trial_name]
        forces = trial_data['forces'][trial_name]
        joint_angles = trial_data['joint_angles'][trial_name]
        torques = trial_data['torques'][trial_name]

        for ((time_pos, x, y), (time_force, fx, fy), (time_angle, angles), (time_torque, torque)) in zip(positions, forces, joint_angles, torques):
            all_data.append({
                'Trial': trial_name,
                'Time': time_pos,  # Assuming time_pos == time_force == time_angle
                'X': x,
                'Y': y,
                'Fx': fx,
                'Fy': fy,
                'Joint_Angles': angles,
                'Torques': torque
            })

    # Convert the list of dictionaries to a DataFrame and save as CSV
    df = pd.DataFrame(all_data)
    file_path = os.path.join(csv_directory, 'all_trials_data.csv')
    df.to_csv(file_path, index=False)
    rospy.loginfo(f"All trials data saved to {file_path}")

def save_te_data():
    csv_directory = os.path.join(os.path.expanduser('~'), 'trajectory_data')
    os.makedirs(csv_directory, exist_ok=True)
    file_path = os.path.join(csv_directory, 'te_data.csv')
    
    df = pd.DataFrame(te_data, columns=['TE_x', 'TE_y'])  # Assuming 2D TE
    df.to_csv(file_path, index=False)
    rospy.loginfo(f"TE data saved to {file_path}")

def save_u_ilc_data():
    csv_directory = os.path.join(os.path.expanduser('~'), 'trajectory_data')
    os.makedirs(csv_directory, exist_ok=True)
    file_path = os.path.join(csv_directory, 'u_ilc_data.csv')
    
    df = pd.DataFrame(u_ilc_data, columns=['u_ilc_x', 'u_ilc_y'])  # Assuming 2D u_ilc
    df.to_csv(file_path, index=False)
    rospy.loginfo(f"u_ilc data saved to {file_path}")

def all_data_received():
    return all(value is not None for key, value in latest_data.items() if key not in ['desired_trajectory', 'external_forces'])

def wait_for_data():
    rospy.loginfo("Waiting for all data to be received...")
    rate = rospy.Rate(100)  # Check at 10 Hz
    while not all_data_received():
        rate.sleep()
    rospy.loginfo("All data received. Starting the trial loop.")

def signal_handler(sig, frame):
    rospy.loginfo("Interrupt signal received. Saving data...")
    save_trial_data()
    save_te_data()
    save_u_ilc_data()
    rospy.signal_shutdown("Shutting down due to interrupt signal.")

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

############################################################################## MAIN ####################################################################################3

def main():
    rospy.init_node('ilc_controller', anonymous=True)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

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