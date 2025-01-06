#!/usr/bin/env python3

import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

def control_joint_positions(joint_positions, path_time=2.0):
    try:
        # Wait for the service to become available
        rospy.wait_for_service('/goal_joint_space_path', timeout=5)

        # Create a service proxy to call the service
        move_joint_service = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        
        # Create the service request
        request = SetJointPositionRequest()
        
        # Define joint names and positions
        request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        request.joint_position.position = joint_positions
        request.path_time = path_time

        # Call the service and get the response
        response = move_joint_service(request)
        if response.is_planned:
            rospy.loginfo("Trajectory planned and executed successfully!")
        else:
            rospy.logwarn("Failed to plan and execute the trajectory.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def control_gripper(position):
    try:
        # Wait for the gripper control service to become available
        rospy.wait_for_service('/goal_tool_control', timeout=5)

        # Create a service proxy to call the service
        gripper_service = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
        
        # Create the service request
        request = SetJointPositionRequest()
        
        # Define gripper name and position
        request.joint_position.joint_name = ["gripper"]
        request.joint_position.position = [position]
        request.path_time = 1.0

        # Call the service and get the response
        response = gripper_service(request)
        if response.is_planned:
            rospy.loginfo("Gripper action executed successfully!")
        else:
            rospy.logwarn("Failed to execute gripper action.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('move_openmanipulator_with_service')

    try:
        joint_positions = [
            [-0.4356505572795868, -0.6044466686248779, -0.13038836419582367, 1.306951642036438],
            [1.4925633668899536, -0.11209710031747818, 0.4555923044681549, 1.0492428541183472],
            [1.5477867126464844, -0.13970875084400177, 0.4801360070705414, 1.1060001850128174],
            [-0.4356505572795868, -0.6044466686248779, -0.13038836419582367, 1.306951642036438],
            [-0.647339940071106, 0.1026602154970169, 0.15033012628555298, 0.9341943264007568],
            [-0.6089903712272644, 0.1026602154970169, 0.15493206679821014, 0.9096506237983704],
        ]

        gripper_open = 0.01  # Define gripper open position
        gripper_close = -0.01  # Define gripper closed position

        for i, position in enumerate(joint_positions):
            if i == 0 or i == 1 or i == 5:
                # Open the gripper at the 1st and 5th positions
                control_gripper(gripper_open)
            elif 2 <= i <= 4:
                # Close the gripper at the 2nd, 3rd, and 4th positions
                control_gripper(gripper_close)

            # Move to the joint position
            control_joint_positions(position, path_time=2.0)
            rospy.sleep(2)

    except rospy.ROSInterruptException:
        pass