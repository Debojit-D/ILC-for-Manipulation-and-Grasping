#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

def control_gripper(open_gripper):
    rospy.init_node('gripper_control')

    # Define the service name
    service_name = '/goal_tool_control'
    rospy.wait_for_service(service_name)

    try:
        # Create a service proxy
        tool_control = rospy.ServiceProxy(service_name, SetJointPosition)
        
        # Create the request object and populate it
        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["gripper"]  # Specify the joint name
        request.joint_position.position = [0.01] if open_gripper else [-0.01]  # Set position

        # Call the service
        response = tool_control(request)

        if response.is_planned:
            rospy.loginfo("Gripper operation successful: %s", "OPENED" if open_gripper else "CLOSED")
        else:
            rospy.logwarn("Gripper operation failed or not planned")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    control_gripper(True)  # Open the gripper
    rospy.sleep(5)  # Pause for 5 seconds
    control_gripper(False)  # Close the gripper
