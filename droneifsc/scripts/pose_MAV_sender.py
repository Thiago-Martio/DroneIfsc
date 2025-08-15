#!/usr/bin/env python3
import rospy
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from pymavlink import mavutil
import math
import time
import numpy as np

def marker_size_rad(larg, alt, distance):
    marker_w = larg         # largura real [m]
    marker_h = alt        # altura real [m]
    D        = distance     # mesma distância que você envia no campo distance [m]

    size_x = 2 * math.atan(marker_w / (2 * D))   # rad
    size_y = 2 * math.atan(marker_h / (2 * D))   # rad
    return (size_x, size_y)
def calculate_3d_distance_numpy(x, y, z):
    """
    Calculates the Euclidean distance between two 3D points using NumPy.

    Args:
        x -> offset from the camera center
        y -> offset from the camera center
        z -> offset from the camera center
    Returns:
        float: The Euclidean distance between the camera and the aruco.
    """
    arpose = [x,y,z]
    p_array = np.array(arpose)
    #origin = np.array([0,0,0])
    distance = np.linalg.norm(p_array)
    return distance


connection_string = 'udp:127.0.0.1:14550'  # ou '/dev/ttyAMA0'
target_num = 0  # ID do alvo
target_type = mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL
frame = mavutil.mavlink.MAV_FRAME_BODY_FRD  # ou LOCAL_NED se preferir
angle_x = 0   # [rad] deslocamento angular no eixo x
angle_y = 0   # [rad] deslocamento angular no eixo y
distance = 0  # [m]
size_x = 0    # [rad] tende a ser ignorado
size_y = 0    # [rad] tende a ser ignorado
position_valid = 1

# Conexão com a FCU
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
print("Conectado à FCU")

def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)

class ArucoPosePublisher:
    def __init__(self):
        rospy.init_node('aruco_pose_publisher', anonymous=True)
        rospy.loginfo("Aruco Pose Publisher initialized!!!!!!!!!!!!!!")
        # Parameters
        self.marker_id = rospy.get_param('~marker_id', 0)  # Default to marker ID 0
        self.board_name = rospy.get_param('~board_name', None)  # None means use marker instead of board
        
        # Configuração do tópico (ajuste conforme seu setup)
        aruco_detections = "/aruco_detections"  # Tópico padrão para sua simulação
        
        # Subscriber
        rospy.Subscriber(aruco_detections, ArucoDetection, self.detection_callback)
        rospy.loginfo(f"Aruco Pose Publisher initialized. Looking for {'board: ' + self.board_name if self.board_name else 'marker ID: ' + str(self.marker_id)}")

        # Publisher
        self.detection_consistency = rospy.Publisher('/aruco_detection_consistency', Bool, queue_size=10)
        self.aruco_pose_publisher = rospy.Publisher('/last_aruco_pose', PoseStamped, queue_size=10)

        self.start_time = rospy.Time.now()
        self.last_detection_time = None
        while not rospy.is_shutdown():
            self.detection_consistency.publish(Bool(data=self.is_detection_consistent(timeout=0.1)))
            rospy.sleep(0.1)
        
    def detection_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header  # Copy the header
        
        found = False

        # Check for board pose if board_name is specified
        if self.board_name:
            for board in msg.boards:
                if board.board_name == self.board_name:
                    pose_stamped.pose = board.pose
                    found = True
                    self.last_detection_time = rospy.Time.now()
                    break
        else:
            # Otherwise check for marker pose
            for marker in msg.markers:
                if marker.marker_id == self.marker_id:
                    pose_stamped.pose = marker.pose
                    found = True
                    self.last_detection_time = rospy.Time.now()
                    break
        
        if found:
            self.aruco_pose_publisher.publish(pose_stamped)
            
            time_usec = int(time.time() * 1e6)  # timestamp atual
            distance = calculate_3d_distance_numpy(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
            angles = marker_position_to_angle(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z)
            sizes = marker_size_rad(0.18, 0.18, distance)
        
            master.mav.landing_target_send(
                time_usec,
                target_num,
                frame,
                angles[0], #x
                angles[1], #y
                distance,
                target_type,
                position_valid,
                sizes[0], #x, firmwares mais novos tende a ignorar
                sizes[1] #y, firmwares mais novos tende a ignorar
            )
            rospy.logdebug(f"LANDING_TARGET enviado a {distance}m")
    def is_detection_consistent(self, timeout=1):
        """
        Check if the detection is consistent based on the last detection time.
        If no detection has been made within the timeout period, return False.
        """
        if self.last_detection_time is None:
            return False
        
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.last_detection_time).to_sec()
        
        if elapsed_time > timeout:
            rospy.logwarn("No detection in the last {} seconds.".format(timeout))
            return False
        
        return True

if __name__ == '__main__':
    try:
        app = ArucoPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
