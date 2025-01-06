#!/usr/bin/env python
import rospy
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations 

def send_pose():
    rospy.init_node('set_kinematics_pose_client')
    
    rospy.wait_for_service('/goal_task_space_path')
    try:
        set_pose = rospy.ServiceProxy('/goal_task_space_path', SetKinematicsPose)
        
        # Define the pose
        target_pose = Pose()
        target_pose.position = Point(x=0.1026  , y=-0.00, z=0.2022)
        
        #target_pose.position = Point(x=0.1096  , y=-0.0, z=0.1358)
        #target_pose.position = Point(x=0.1896  , y=-0.0, z=0.1358)
        #target_pose.position = Point(x=0.1896  , y=-0.0, z=0.1858)

        #Start_Position x=0.09995 y=-0.00390 z=0.06589 Orientation x=0.0000 y=1.4588 z=-0.04448
        #End_Position x=0.15 y=-0.00390 z=0.06589 Orientation x=0.0000 y=1.4588 z=-0.04448
        
        #New_End_Position #End_Position x=0.20 y=-0.00390 z=0.20 Orientation x=0.0000 y=1.4588 z=-0.04448


        # Define the Euler angles (roll, pitch, yaw)
        roll = 0.0  # in radians
        pitch = 0.00 # in radians
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

if __name__ == "__main__":
    send_pose()
