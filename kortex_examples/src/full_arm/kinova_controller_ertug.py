#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time

from kortex_driver.srv import *
from kortex_driver.msg import *
from geometry_msgs.msg import Twist


class KinovaControllerErtug:
    def __init__(self):
        print('START INIT')

        try:
            rospy.init_node('kinova_controller_ertug_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            #self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            #self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)
            self.degrees_of_freedom = rospy.get_param("/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param( "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # self.feedback_topic_sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback, self.cb_feedback_topic)
            self.feedback_topic_sub = rospy.Subscriber("/base_feedback", BaseCyclic_Feedback, self.cb_feedback_topic)
            self.go_to_topic_sub = rospy.Subscriber("/in/cartesian_go_to_pose", Twist, self.cb_go_to_topic)

            self.kinova_vel_cmd_pub = rospy.Publisher('/in/cartesian_velocity', TwistCommand, queue_size=30) ## position.x
            self.kinova_current_pose_pub = rospy.Publisher('/in/cartesian_current_pose', Twist, queue_size=30) ## position.x
            # self.example_home_the_robot()

            # # Init the action topic subscriber
            # self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            # self.last_action_notif_type = None

            # self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            # self.last_action_notif_type = None

            # # Init the services
            # clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            # rospy.wait_for_service(clear_faults_full_name)
            # self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            # read_action_full_name = '/' + self.robot_name + '/base/read_action'
            # rospy.wait_for_service(read_action_full_name)
            # self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            # execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            # rospy.wait_for_service(execute_action_full_name)
            # self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            # set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            # rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            # self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            # send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            # rospy.wait_for_service(send_gripper_command_full_name)
            # self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            # activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            # rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            # self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            # get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            # rospy.wait_for_service(get_product_configuration_full_name)
            # self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            # validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            # rospy.wait_for_service(validate_waypoint_list_full_name)
            # self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
 
 
            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber( "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            self.action_topic_sub = rospy.Subscriber( "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name =   '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name =   '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name =   '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name =   '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            send_gripper_command_full_name =   '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name =   '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name =   '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name =   '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
 
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def cb_feedback_topic(self, msg):
        #global feedback_msg
        self.feedback_msg=msg
        #print("feeddback_callback_fcn")
        #print("msg",msg.base.pose)
        #self.feedback_msg = msg

    def cb_go_to_topic(self, msg):
        self.go_to_pose=msg #in twist format

    


    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        waypoint = Waypoint()
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
        waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

        return waypoint

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        self.last_action_notif_type = None
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    def example_send_cartesian_pose(self):
        self.last_action_notif_type = None
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)

        # Possible to execute waypointList via execute_action service or use execute_waypoint_trajectory service directly
        req = ExecuteActionRequest()
        trajectory = WaypointList()

        trajectory.waypoints.append(
            self.FillCartesianWaypoint(
                feedback.base.commanded_tool_pose_x,
                feedback.base.commanded_tool_pose_y,
                feedback.base.commanded_tool_pose_z + 0.10,
                feedback.base.commanded_tool_pose_theta_x,
                feedback.base.commanded_tool_pose_theta_y,
                feedback.base.commanded_tool_pose_theta_z,
                0)
        )

        trajectory.duration = 0
        trajectory.use_optimal_blending = False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)

        # Call the service
        rospy.loginfo("Sending the robot to the cartesian pose...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_joint_angles(self):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoint = Waypoint()
        angularWaypoint = AngularWaypoint()

        if self.degrees_of_freedom ==7:
            angularWaypoint.angles.append(321.66033935546875)
            angularWaypoint.angles.append(260.0)
            angularWaypoint.angles.append(358.0)
            angularWaypoint.angles.append(138.0)
            angularWaypoint.angles.append(333.0)
            angularWaypoint.angles.append(23.0)
            angularWaypoint.angles.append(309.0)
        print("degrees_of_freedom:",self.degrees_of_freedom)
        print("angularWaypoint:",angularWaypoint)



        # Angles to send the arm to vertical position (all zeros)
        # for _ in range(self.degrees_of_freedom):
        #     if _ ==0 :
        #         angularWaypoint.angles.append(321.66033935546875)
        #     # if _ ==2 :
        #     #     angularWaypoint.angles.append(-10)
        #     else:
        #         angularWaypoint.angles.append(0.0)


        # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
        # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
        angular_duration = 0
        angularWaypoint.duration = angular_duration

        # Initialize Waypoint and WaypointList
        waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoint)

        try:
            res = self.validate_waypoint_list(trajectory)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ValidateWaypointList")
            return False

        error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
        MAX_ANGULAR_DURATION = 30

        while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
            angular_duration += 1
            trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

        if (angular_duration == MAX_ANGULAR_DURATION) :
            # It should be possible to reach position within 30s
            # WaypointList is invalid (other error than angularWaypoint duration)
            rospy.loginfo("WaypointList is invalid")
            return False

        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
        # Send the angles
        rospy.loginfo("Sending the robot vertical...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True

    def example_cartesian_waypoint_action(self):
        self.last_action_notif_type = None

        req = ExecuteActionRequest()
        trajectory = WaypointList()

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
        
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.439,  0.194,  0.448, 90.6, -1.0, 150, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.200,  0.150,  0.400, 90.6, -1.0, 150, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.350,  0.050,  0.300, 90.6, -1.0, 150, 0))
        else:
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.0,   0.5,  90, 0, 90, 0))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.0,   0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.61, 0.22,  0.4,  90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.7,  0.48,  0.33, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.63, -0.22, 0.45, 90, 0, 90, 0.1))
            trajectory.waypoints.append(self.FillCartesianWaypoint(0.65, 0.05,  0.45, 90, 0, 90, 0))
        
        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        
        # Call the service
        rospy.loginfo("Executing Kortex action ExecuteWaypointTrajectory...")
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call action ExecuteWaypointTrajectory")
            return False
        else:
            return self.wait_for_action_end_or_abort()

    def saturation(self,val, lo, hi):
        #def __init__(self, val, lo, hi):
        #    self.real, self.lo, self.hi = val, lo, hi
        if lo < val < hi:
            return val
        if val <lo:
            return lo
        if val > hi: 
            return hi 

    def main(self):
        # success &= self.example_home_the_robot()
        rate = rospy.Rate(100) # 100hz
        # For testing purposes
        print('I AM ERTUG, HERE I AM1')
        success = self.is_init_success
        # success &= self.example_home_the_robot()

        print('I AM ERTUG, HERE IS - success:', success)
        #global feedback_msg
        self.twist_command_msg = TwistCommand()
        self.feedback_msg = BaseCyclic_Feedback()
        self.go_to_pose = Twist()
        self.current_pose_msg = Twist()

#example delivery:
        #x_ideal = -0.45 #negative values goes upward #0 is center
        #y_ideal = 0.1
        #z_ideal = 1.0
        #x_theta_ideal= -5.0 #90 looking downward 0:looking in front, -90 looking upward
        #y_theta_ideal= 10.0 #
        #z_theta_ideal= 90.0 


        # x_ideal = -0.15 #goes upward #0 is center
        # y_ideal = 0.0
        # z_ideal = 0.95
        # x_theta_ideal= -5.0 #90 looking downward 0:looking in front, -90 looking upward
        # y_theta_ideal= 0.0 #
        # z_theta_ideal= 90.0 

        


        # x_ideal = -0.15 #goes upward #0 is center
        # y_ideal = 0.1
        # z_ideal = 0.85
        # x_theta_ideal= 45.0 #90 looking downward 0:looking in front, -90 looking upward
        # y_theta_ideal= 0.0 #
        # z_theta_ideal= 100.0 

# use this to take the SPOT out of the camera scene: 

        #x_ideal = -0.0
        #y_ideal = 0.05
        #z_ideal = 0.6
        #x_theta_ideal= 60.0 #90 looking downward 0:looking in front, -90 looking upward
        #y_theta_ideal= 20.0 #
        #z_theta_ideal= 135.0 

        #self.go_to_pose.linear.x = -0.45
        #self.go_to_pose.linear.y = 0.1
        #self.go_to_pose.linear.z = 1.0
        #self.go_to_pose.angular.x = -5.0
        #self.go_to_pose.angular.y = 10.0
        #self.go_to_pose.angular.z = 90.0

        #initial setup:
        self.go_to_pose.linear.x = -0.0
        self.go_to_pose.linear.y = 0.05
        self.go_to_pose.linear.z = 0.6
        self.go_to_pose.angular.x = 60.0
        self.go_to_pose.angular.y = 20.0
        self.go_to_pose.angular.z = 135.0

        x_ideal = self.go_to_pose.linear.x #negative values goes upward #0 is center
        y_ideal = self.go_to_pose.linear.y
        z_ideal = self.go_to_pose.linear.z
        x_theta_ideal= self.go_to_pose.angular.x #90 looking downward 0:looking in front, -90 looking upward
        y_theta_ideal= self.go_to_pose.angular.y
        z_theta_ideal= self.go_to_pose.angular.z
        # self.counter=1
    
        k_p_angular = 0.02

        if success:



        #     #*******************************************************************************
        #     # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
        #     #*******************************************************************************
            
        #     #*******************************************************************************
        #     # Activate the action notifications
            success &= self.example_subscribe_to_a_robot_notification()
        #     #*******************************************************************************
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.0)
        #     #*******************************************************************************
        #     # Move the robot to the Home position with an Action
        #     success &= self.example_home_the_robot()
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Example of gripper command
        #     # Let's fully open the gripper
        #     if self.is_gripper_present:
        #         success &= self.example_send_gripper_command(0.0)
        #     else:
        #         rospy.logwarn("No gripper is present on the arm.")  
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Set the reference frame to "Mixed"
        #     success &= self.example_set_cartesian_reference_frame()

        #     # Example of cartesian pose
        #     # Let's make it move in Z
        #     success &= self.example_send_cartesian_pose()
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Example of angular position
        #     # Let's send the arm to vertical position
            success &= self.example_send_joint_angles()
            # success &= self.example_send_joint_angles()
            # success &= self.example_send_joint_angles()
            #success &= self.example_send_joint_angles()
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Example of gripper command
        #     # Let's close the gripper at 50%
        #     if self.is_gripper_present:
        #         success &= self.example_send_gripper_command(0.5)
        #     else:
        #         rospy.logwarn("No gripper is present on the arm.")    
        #     #*******************************************************************************
        
        #     #*******************************************************************************
        #     # Move the robot to the Home position with an Action
        #     success &= self.example_home_the_robot()
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Example of waypoint
        #     # Let's move the arm
        #     success &= self.example_cartesian_waypoint_action()

        #     #*******************************************************************************
        #     # Move the robot to the Home position with an Action
        #     success &= self.example_home_the_robot()
        #     #*******************************************************************************

            # rospy.sleep(5.0)


            #print('I AM ERTUG, HERE IS2 - success:', success)
            saturation_lim=0.05
            #saturation_lim=0.07
            saturation_lim_angular=0.08
            #saturation_lim_angular=0.12
            #saturation_lim=0.03
            #self.twist_command_msg.twist.linear_z = 0.01

            while not rospy.is_shutdown():
                #msg_all = Joy()
                #print('I AM ERTUG, HERE IS2 - success:', success)
                #print('feedback_msg', self.feedback_msg.base.tool_pose_z)
                #print('twist_command_msg', self.twist_command_msg.twist.linear_z)

                x_ideal = self.go_to_pose.linear.x #negative values goes upward #0 is center
                y_ideal = self.go_to_pose.linear.y
                z_ideal = self.go_to_pose.linear.z
                x_theta_ideal= self.go_to_pose.angular.x #90 looking downward 0:looking in front, -90 looking upward
                y_theta_ideal= self.go_to_pose.angular.y
                z_theta_ideal= self.go_to_pose.angular.z
                error_x = x_ideal-self.feedback_msg.base.tool_pose_x
                error_y = y_ideal-self.feedback_msg.base.tool_pose_y
                error_z = z_ideal-self.feedback_msg.base.tool_pose_z
                error_x_theta = (x_theta_ideal-self.feedback_msg.base.tool_pose_theta_x)*k_p_angular
                error_y_theta = (y_theta_ideal-self.feedback_msg.base.tool_pose_theta_y)*k_p_angular
                error_z_theta = (z_theta_ideal-self.feedback_msg.base.tool_pose_theta_z)*k_p_angular
                #print("error_x_theta",error_x_theta)
                #print("error_y_theta",error_y_theta)
                #print("error_z_theta",error_z_theta)
                print("self.go_to_pose",self.go_to_pose)
                # print("self.counter",self.counter)
                # self.counter=self.counter+1



                #print("error_z",error_z)
                self.twist_command_msg.twist.linear_x = self.saturation(error_x,-saturation_lim,saturation_lim)
                self.twist_command_msg.twist.linear_y = self.saturation(error_y,-saturation_lim,saturation_lim)
                self.twist_command_msg.twist.linear_z = self.saturation(error_z,-saturation_lim,saturation_lim)
                #print("self.twist_command_msg.twist.linear_z",self.twist_command_msg.twist.linear_z)

                self.twist_command_msg.twist.angular_x = self.saturation(error_x_theta,-saturation_lim_angular,saturation_lim_angular)
                self.twist_command_msg.twist.angular_y = self.saturation(error_y_theta,-saturation_lim_angular,saturation_lim_angular)
                self.twist_command_msg.twist.angular_z = self.saturation(error_z_theta,-saturation_lim_angular,saturation_lim_angular)
                # if (self.counter >1000) and (self.counter <2000):
                #     success &= self.example_send_joint_angles()
                # else:
                self.kinova_vel_cmd_pub.publish(self.twist_command_msg)
            

                self.current_pose_msg.linear.x = self.feedback_msg.base.tool_pose_x
                self.current_pose_msg.linear.y = self.feedback_msg.base.tool_pose_y
                self.current_pose_msg.linear.z = self.feedback_msg.base.tool_pose_z
                self.current_pose_msg.angular.x = self.feedback_msg.base.tool_pose_theta_x
                self.current_pose_msg.angular.y = self.feedback_msg.base.tool_pose_theta_y
                self.current_pose_msg.angular.z = self.feedback_msg.base.tool_pose_theta_z

                self.kinova_current_pose_pub.publish(self.current_pose_msg)

                # if self.is_gripper_present:
                #     success &= self.example_send_gripper_command(0.0)
                # else:
                #     rospy.logwarn("No gripper is present on the arm.")  


                rate.sleep()



        # try:
        #     rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        # except:
        #     pass

        # if success:
        #     #*******************************************************************************
        #     # Make sure to clear the robot's faults else it won't move if it's already in fault
        #     success &= self.example_clear_faults()
        #     #*******************************************************************************
            
        #     #*******************************************************************************
        #     # Activate the action notifications
        #     success &= self.example_subscribe_to_a_robot_notification()
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Move the robot to the Home position with an Action
        #     success &= self.example_home_the_robot()
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Example of gripper command
        #     # Let's fully open the gripper
        #     if self.is_gripper_present:
        #         success &= self.example_send_gripper_command(0.0)
        #     else:
        #         rospy.logwarn("No gripper is present on the arm.")  
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Set the reference frame to "Mixed"
        #     success &= self.example_set_cartesian_reference_frame()

        #     # Example of cartesian pose
        #     # Let's make it move in Z
        #     success &= self.example_send_cartesian_pose()
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Example of angular position
        #     # Let's send the arm to vertical position
        #     success &= self.example_send_joint_angles()
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Example of gripper command
        #     # Let's close the gripper at 50%
        #     if self.is_gripper_present:
        #         success &= self.example_send_gripper_command(0.5)
        #     else:
        #         rospy.logwarn("No gripper is present on the arm.")    
        #     #*******************************************************************************
        
        #     #*******************************************************************************
        #     # Move the robot to the Home position with an Action
        #     success &= self.example_home_the_robot()
        #     #*******************************************************************************

        #     #*******************************************************************************
        #     # Example of waypoint
        #     # Let's move the arm
        #     success &= self.example_cartesian_waypoint_action()

        #     #*******************************************************************************
        #     # Move the robot to the Home position with an Action
        #     success &= self.example_home_the_robot()
        #     #*******************************************************************************

        # # For testing purposes
        # rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

        # if not success:
        #     rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    print('I AM ERTUG, HERE I AM:')
    ex = KinovaControllerErtug()
    ex.main()
