#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import rospy
from cv_bridge import CvBridge, CvBridgeError
import yaml
import time

class CameraHandler:
    def __init__(self):
        rospy.init_node('camera_handler')

        boot_ts = time.clock_gettime(time.CLOCK_BOOTTIME)
        epoch_ts = time.clock_gettime(time.CLOCK_REALTIME)
        self.boot_epoch_offset = epoch_ts - boot_ts  # em segundos
        # Parameters
        self.image_topic = '/camera/image_raw'
        self.camera_info_topic = '/camera/camera_info'
        self.frame_id = 'camera_frame'
        self.ost_file = 'ost.yaml'  # Path to the camera calibration file
        self.shm_path = '/dev/shm/rpicam/cam.mjpeg'  # <-- arquivo no /dev/shm
        # Initialize camera capture
        rospy.loginfo("Initializing camera...")
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Publishers for image and camera info
        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=10)
        
        # Create and publish camera info (dummy values for demonstration)
        self.camera_info = self.get_camera_info()
        
        # Initialize VideoCapture
        rospy.loginfo("Opening shared memory camera stream: %s", self.shm_path)
        self.cap = cv2.VideoCapture(self.shm_path)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open video stream from %s", self.shm_path)
        
        rospy.loginfo("Camera Handler initialized with topics: %s and %s", self.image_topic, self.camera_info_topic)

        # Main loop
        rate = rospy.Rate(20)  # 20 Hz
        while not rospy.is_shutdown():
            self.publish_camera_data()
            rate.sleep()
    def publish_camera_data(self):
        ret, img = self.cap.read()
        if not ret or img is None:
            rospy.logwarn("No frame captured from shared memory stream.")
            return
        # aqui você teria algo como sensor_timestamp (ns desde boot)
        # se o driver te dá esse valor, por ex. self.camera.timestamp
        # senão, pode usar CLOCK_BOOTTIME direto:
        sensor_ts_boot = time.clock_gettime(time.CLOCK_BOOTTIME)  

        # converte para tempo de parede (epoch)
        sensor_ts_epoch = sensor_ts_boot + self.boot_epoch_offset  

        # cria stamp ROS
        secs = int(sensor_ts_epoch)
        nsecs = int((sensor_ts_epoch - secs) * 1e9)
        stamp = rospy.Time(secs, nsecs)
        #cvt to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        try:
            ros_image = self.bridge.cv2_to_imgmsg(gray, encoding="mono8") 
            ros_image.header.stamp = stamp
            ros_image.header.frame_id = self.frame_id
            # Publish
            self.image_pub.publish(ros_image)
            self.camera_info.header = ros_image.header
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
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        rospy.loginfo("Camera Handler node shutting down.")
if __name__ == '__main__':
    try:
        camera_handler_node = CameraHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        camera_handler_node.__del__()
        pass
