#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import rospy
from cv_bridge import CvBridge, CvBridgeError
import yaml

class CameraHandler:
    def __init__(self):
        rospy.init_node('camera_handler')
        
        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
        self.frame_id = rospy.get_param('~frame_id', 'camera_frame')
        self.ost_file = rospy.get_param('~ost_file', '/home/thiago/catkin_ws/src/droneifsc/scripts/ost.yaml')  # Path to the camera calibration file
        
        # Initialize camera capture
        rospy.loginfo("Initializing camera...")
        self.cvcam = cv2.VideoCapture(0)  # Open default camera
        if not self.cvcam.isOpened():
            rospy.logerr("Could not open camera.")
            return
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Publishers for image and camera info
        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=10)
        
        # Create and publish camera info (dummy values for demonstration)
        self.camera_info = self.get_camera_info()
        
        rospy.loginfo("Camera Handler initialized with topics: %s and %s", self.image_topic, self.camera_info_topic)
        while not rospy.is_shutdown():
            self.publish_camera_data()
    def publish_camera_data(self):
        # Simulate capturing an image (replace with actual camera capture code)
        try:
            # Create a dummy image (for demonstration purposes)
            sucess, cvimage = self.cvcam.read()  # Replace with actual image capture
            if cvimage is None or not sucess:
                rospy.logerr("Failed to read image.")
                return
            
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cvimage, encoding="bgr8")
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = self.frame_id
            
            # Publish the image
            self.image_pub.publish(ros_image)

            self.camera_info.header = ros_image.header
            #Publish the camera info
            self.camera_info_pub.publish(self.camera_info)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

    def get_camera_info(self):
        try:
            with open(self.ost_file, 'r') as file:
                data = yaml.safe_load(file)
                camera_info = CameraInfo()

                camera_info.width = data['image_width']
                camera_info.height = data['image_height']

                camera_info.distortion_model = data['distortion_model']

                camera_info.D = data["distortion_coefficients"]["data"]  # Distortion coefficients
                camera_info.K = data["camera_matrix"]["data"]  # Intrinsic parameters
                camera_info.R = data["rectification_matrix"]["data"] # Rotation matrix
                camera_info.P = data["projection_matrix"]["data"] # Projection matrix
                
                camera_info.binning_x = 0
                camera_info.binning_y = 0

                camera_info.roi.x_offset = 0
                camera_info.roi.y_offset = 0
                camera_info.roi.height = 0
                camera_info.roi.width = 0
                camera_info.roi.do_rectify = False

                return camera_info
            print(data)
        except FileNotFoundError:
            print(f"Error: The file {self.ost_file} was not found.")
        except yaml.YAMLError as exc:
            print(f"Error parsing YAML: {exc}")
    def __del__(self):
        if self.cvcam.isOpened():
            self.cvcam.release()
        rospy.loginfo("Camera Handler node shutting down.")
if __name__ == '__main__':
    try:
        camera_handler_node = CameraHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        camera_handler_node.__del__()
        pass
