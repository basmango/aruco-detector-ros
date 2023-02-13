import rospy
import cv2
import cv_bridge
import numpy as np
from cv2 import aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image


class ArucoDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/video_frames', Image, self.image_callback)
        self.image_pub = rospy.Publisher('/detected_aruco_frames', Image, queue_size=10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters =cv2.aruco.DetectorParameters() 
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

        if len(corners) > 0:
            cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    aruco_detector = ArucoDetector()
    rospy.spin()
