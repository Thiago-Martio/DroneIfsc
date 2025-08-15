#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np

import mavros_msgs.srv as mavros_srv

class NavigatorNode:
    def __init__(self):
        rospy.init_node('NavigatorNode', anonymous=True)

        # Wait for the services to be available
        rospy.wait_for_service('/mavros/setpoint_position/mav_frame')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/vehicle_info_get')
        rospy.loginfo("All required services are available.")
        # MAX PRIORITY
        self.setpoint_mav_frame = rospy.ServiceProxy('/mavros/setpoint_position/mav_frame', mavros_srv.SetMavFrame)
        self.setpoint_frame_response = self.setpoint_mav_frame(9)
        if not self.setpoint_frame_response.success:
            rospy.logerr("Failed to set MAV_FRAME. Exiting.")
            rospy.signal_shutdown("Failed to set MAV_FRAME.")

        # Parameters
        self.desired_altitude = rospy.get_param('~desired_altitude', 1.5)  # Altitude in meters
        self.flying = False
        self.current_pose = PoseStamped()
        self.last_detection_pose = PoseStamped()
        self.is_detection_consistent = False
        self.desired_x = rospy.get_param('~desired_x', 1.0)
        self.desired_y = rospy.get_param('~desired_y', 1.0)
        self.desired_z = rospy.get_param('~desired_z', 0.0)

        # Configuração do tópico (ajuste conforme seu setup)
        drone_pose_topic = "/mavros/local_position/pose"
        setpoint_local_topic = "/mavros/setpoint_position/local"  # Tópico padrão para sua simulação
        consistency_topic = "/aruco_detection_consistency" 
        last_pose_topic = "/last_aruco_pose"
        # Subscribers
        rospy.Subscriber(drone_pose_topic, PoseStamped, self.drone_pose_callback)
        rospy.Subscriber(consistency_topic, Bool, self.detection_consistency_callback)
        rospy.Subscriber(last_pose_topic, PoseStamped, self.last_pose_callback)
        # Publishers
        self.pose_sender = rospy.Publisher(setpoint_local_topic, PoseStamped, queue_size=10)
        

    def arm_and_takeoff(self):
        vehicle_info_service = rospy.ServiceProxy('/mavros/vehicle_info_get', mavros_srv.VehicleInfoGet)
        arm_service = rospy.ServiceProxy('/mavros/cmd/arming', mavros_srv.CommandBool)
        takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_srv.CommandTOL)

        vehicle_info_response = vehicle_info_service(0, 0, False)

        if vehicle_info_response.vehicles[0].mode != 'GUIDED':
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', mavros_srv.SetMode)
            set_mode_response = set_mode_service(custom_mode='GUIDED')
            if set_mode_response.mode_sent:
                rospy.loginfo("Switched to GUIDED mode.")
            else:
                rospy.logerr("Failed to switch to GUIDED mode.")
                return
        try:
            # Arm the vehicle
            arm_response = arm_service(True)
            if arm_response.success:
                rospy.loginfo("Vehicle armed successfully.")
            else:
                rospy.logerr("Failed to arm the vehicle.")
                return
            
            # Takeoff to a specified altitude
            takeoff_response = takeoff_service(0, 0, 0, 0, self.desired_altitude)  # Altitude in meters
            if takeoff_response.success:
                rospy.loginfo("Takeoff command sent successfully.")
                self.flying = True
                while self.current_pose.pose.position.z + self.desired_altitude*0.1 <= self.desired_altitude:
                    rospy.loginfo(f"Current altitude: {self.current_pose.pose.position.z}m, waiting to reach {self.desired_altitude}m")
                    rospy.sleep(0.1)
                rospy.loginfo(f"Reached desired altitude: {self.desired_altitude}m")
            else:
                rospy.logerr("Failed to send takeoff command.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    def goto_desired_position(self, dx, dy, dz, tolerance=0.1, timeout=30):
        if not self.flying:
            rospy.logwarn("Vehicle is not flying. Cannot send position setpoint.")
            return False
        
        # Registrar estado inicial
        start_position = np.array([
            [self.current_pose.pose.position.x],
            [self.current_pose.pose.position.y],
            [self.current_pose.pose.position.z]
        ])
        start_time = rospy.Time.now()
        last_position = start_position
        last_change_time = start_time
        
        # Publicar comando
        position_stamped = PoseStamped()
        position_stamped.pose.position.x = dx
        position_stamped.pose.position.y = dy
        position_stamped.pose.position.z = dz
        position_stamped.pose.orientation.w = 1.0
        self.pose_sender.publish(position_stamped)
        
        # Calcular distância esperada
        expected_distance = np.linalg.norm([dx, dy, dz])
        
        while not rospy.is_shutdown():
            current_position = np.array([
                [self.current_pose.pose.position.x],
                [self.current_pose.pose.position.y],
                [self.current_pose.pose.position.z]
            ])
            
            # Calcular progresso
            displacement = current_position - start_position
            distance_traveled = np.linalg.norm(displacement)
            progress = distance_traveled / expected_distance
            
            # Verificar se chegou
            if distance_traveled >= expected_distance - tolerance:
                rospy.loginfo(f"Reached destination. Traveled: {distance_traveled:.2f}m")
                return True
            
            # Verificar estagnação
            position_change = np.linalg.norm(current_position - last_position)
            if position_change < tolerance/2:
                if (rospy.Time.now() - last_change_time).to_sec() > 2.0:
                    rospy.logwarn(f"Stagnated at {distance_traveled:.2f}m from start")
                    return False
            else:
                last_position = current_position
                last_change_time = rospy.Time.now()
            
            # Verificar timeout
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn(f"Timeout after {timeout} seconds. Traveled: {distance_traveled:.2f}m")
                return False
                
            rospy.loginfo(f"Progress: {progress*100:.1f}% ({distance_traveled:.2f}m/{expected_distance:.2f}m)")
            rospy.sleep(0.2)
    def seek_for_consistency_and_land(self):
        """
        Continuously check for detection consistency and update the last detected pose.
        """
        rospy.sleep(2)
        while not self.is_detection_consistent and not rospy.is_shutdown():
            rospy.loginfo("Seeking for detection consistency...")
            self.goto_desired_position(self.last_detection_pose.pose.position.x, self.last_detection_pose.pose.position.y, 0)  # Stay at current position
            rospy.sleep(2.0)  # Adjust the sleep time as needed
        rospy.loginfo("Detection is consistent. Proceeding with land.")
        self.land()
    def land(self):
        """
        Land the vehicle safely.
        """
        
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', mavros_srv.SetMode)
        try:
            land_response = set_mode_service(custom_mode='LAND')
            if not land_response.mode_sent:
                rospy.logerr("Failed to switch to LAND mode.")
                return
            else:
                rospy.logerr("Failed to send landing command.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    def drone_pose_callback(self, msg):
        self.current_pose = msg
    def detection_consistency_callback(self, msg):
        """
        Callback to handle detection consistency messages.
        """
        self.is_detection_consistent = msg.data
        if self.is_detection_consistent:
            rospy.loginfo("Detection is consistent.")
        else:
            rospy.logwarn("Detection is inconsistent. Seeking for consistency...")
    def last_pose_callback(self, msg):
        """
        Callback to handle the last detected pose.
        """
        self.last_detection_pose = msg
        rospy.loginfo(f"Last detected pose updated: {self.last_detection_pose.pose.position.x}, {self.last_detection_pose.pose.position.y}, {self.last_detection_pose.pose.position.z}")
if __name__ == '__main__':
    navigator_node = NavigatorNode()
    try:
        rospy.loginfo("Arming and taking off...")
        navigator_node.arm_and_takeoff()
        navigator_node.goto_desired_position(navigator_node.desired_x, navigator_node.desired_y, navigator_node.desired_z)  # Pass None or a specific position if needed
        navigator_node.seek_for_consistency_and_land()  # Ensure detection consistency
        rospy.spin()
    except rospy.ROSInterruptException:
        navigator_node.land()
    finally:  # Garante que land()
        if navigator_node.flying:
            navigator_node.land()
        rospy.loginfo("Node shutdown")
