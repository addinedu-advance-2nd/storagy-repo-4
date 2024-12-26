import cv2
import socket
import pickle
import struct
import numpy as np
import math

import rclpy as rp
from rclpy.node import Node
# from control_msgs.msg import ArucoPose

ip = '192.168.0.179'
port = 50001       

def rotMat2degree(rotation_matrix):
    sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = 0

    x = math.degrees(x)
    y = math.degrees(y)
    z = math.degrees(z)
    
    return x, y, z

# class ArucoPosePublisher(Node):
#     def __init__(self):
#         super().__init__('aruco_pose_publisher')
#         self.publisher = self.create_publisher(ArucoPose, '/aruco_pose', 10)
    
#     def send_aruco_pose(self, id, tvecs, angles):
#         msg = ArucoPose()
#         msg.id = id
#         msg.tvecs = tvecs
#         msg.angles = angles
#         self.publisher.publish(msg)


rp.init()
# aruco_pose_node = ArucoPosePublisher()

calib_data_path = "vision/calib_data/MultiMatrix.npz"

with np.load(calib_data_path) as calib_data:
    print(calib_data.files)

    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]
    r_vectors = calib_data["rVector"]
    t_vectors = calib_data["tVector"]

marker_length = 0.04
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
    client_socket.connect((ip, port))
    print("connection success")

    data_buffer = b""
    data_size = 4

    while True:
        while len(data_buffer) < data_size:
            data_buffer += client_socket.recv(4096)
        
        packed_data_size = data_buffer[:data_size]
        data_buffer = data_buffer[data_size:]

        frame_size = struct.unpack(">L", packed_data_size)[0]

        while len(data_buffer) < frame_size:
            data_buffer += client_socket.recv(4096)
        
        frame_data = data_buffer[:frame_size]
        data_buffer = data_buffer[frame_size:]
        print("received frame size : {} bytes".format(frame_size))

        frame = pickle.loads(frame_data)

        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

        corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)   # corners = 2d 픽셀 좌표

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, cam_mat, dist_coef)

            for i in range(len(ids)):
                cv2.drawFrameAxes(frame, cam_mat, dist_coef, rvecs[i], tvecs[i], marker_length)
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                rotation_degree = rotMat2degree(rotation_matrix)

                print(f'Rotation : {rotation_matrix}, Translation : {tvecs[i]}')
                print(f'angle : {rotation_degree}')
                print(rvecs)

                # aruco_pose_node.send_aruco_pose(id, tvecs, rotation_degree)

        cv2.imshow('Frame', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    
    cv2.destroyAllWindows()


