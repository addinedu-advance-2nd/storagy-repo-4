import rclpy as rp
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

# from control_msgs.msg import RobotArriveState, RobotRecevieMoving, RobotRequestMoving

import math
import time

from cv_bridge import CvBridge
import cv2
from cv2 import aruco

import socket 
import struct
import pickle

ip = "192.168.0.179"
port = 50001

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('Camera_Image_Subscriber')
        

        self.bridge = CvBridge()
        self.cv_image = None

        # aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        # parameters = aruco.DetectorParameters() 
        # self.detector = aruco.ArucoDetector(aruco_dict, parameters)

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((ip, port))
        self.server_socket.listen(10)
        print('wating client...')

        self.client_socket, address = self.server_socket.accept()
        print("ip address of client :", address[0])

        self.data_buffer = b""

        self.data_size = 4

        self.image_subscriber = self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, 10)

        self.cut_flag = 0   # down frame
        

    def camera_callback(self, msg):
        if self.cut_flag < 5:   # down frame
            self.cut_flag += 1
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # cv_image = cv2.resize(cv_image, (320, 240))  
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # corners, ids, rejectedCandidates = self.detector.detectMarkers(gray)
        print(gray.shape)

        ret, frame = cv2.imencode('.jpg', gray, [cv2.IMWRITE_JPEG_QUALITY, 70])

        frame = pickle.dumps(frame)
        print("frame size : {} byte".format(len(frame)))

        self.client_socket.sendall(struct.pack(">L", len(frame)) + frame)
        self.cut_flag = 0


def main(args=None):

    rp.init(args=args)
    camera_subscriber = ImageSubscriber()
    
    executor = MultiThreadedExecutor()

    executor.add_node(camera_subscriber)
 

    try:
        executor.spin()

    finally:
        executor.shutdown()
        camera_subscriber.destroy_node()
        rp.shutdown()
        camera_subscriber.client_socket.close()
        camera_subscriber.server_socket.close()


if __name__ == '__main__':
    main()