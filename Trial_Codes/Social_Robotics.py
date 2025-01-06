#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest, SetJointPosition, SetJointPositionRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tf_tr

def euler_to_quaternion(roll, pitch, yaw):
    # Convert Euler angles (roll, pitch, yaw) into quaternion
    quaternion = tf_tr.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*quaternion)

def control_gripper(joint_position):
    # Define the service name for gripper control
    service_name = '/goal_tool_control'
    rospy.wait_for_service(service_name)
    try:
        tool_control = rospy.ServiceProxy(service_name, SetJointPosition)
        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["gripper"]
        request.joint_position.position = [joint_position]  # Set position
        response = tool_control(request)
        if response.is_planned:
            rospy.loginfo("Gripper operation successful: %s", "OPENED" if joint_position > 0 else "CLOSED")
        else:
            rospy.logwarn("Gripper operation failed or not planned")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def send_pose(poses_with_orientations):
    rospy.init_node('pose_gripper_control_node')
    
    # Wait for the kinematics pose service to become available
    rospy.wait_for_service('/goal_task_space_path')
    set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)

    # Main loop for setting poses and controlling gripper
    for position, orientation, gripper_state, delay in poses_with_orientations:
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
                rospy.sleep(delay)  # Variable delay after reaching the pose
                if gripper_state != "unchanged":
                    control_gripper(0.01 if gripper_state == "open" else -0.01)
            else:
                rospy.loginfo("Failed to send pose.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        
        rospy.sleep(2)  # Sleep to maintain loop rate

if __name__ == "__main__":
    rospy.sleep(5)
    poses_with_orientations = [
        #((0.0193, 0.1328, 0.0701), (0.0, 1.3169, 1.3155), "unchanged", 0.2),
        #((0.0193, 0.1328, 0.0401), (0.0, 1.3169, 1.3155), "close", 1.5),
        ((0.0193, 0.1328, 0.05), (0.0, 1.3169, 1.3155), "unchanged", 0.1),
        #((0.167, -0.092, 0.075), (0.0, 1.40, -0.541), "open", 3),
        #((0.167, -0.092, 0.10), (0.0, 1.40, -0.541), "open", 0.1),
        #((0.14, -0.01, 0.10), (0.0, 1.40, -0.541), "unchanged", 0.1),
    ]
    send_pose(poses_with_orientations)
