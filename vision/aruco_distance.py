import cv2 as cv
from cv2 import aruco
import numpy as np

import rclpy as rp
from rclpy.node import Node
# [sensor_msgs/msg/Image를 opencv 형식으로 변환]
from cv_bridge import CvBridge    #패키지 import


# class VisionPublisher(Node):
#     def __init__(self):
#         super().__init__('vision_publiser_node')



# self.bridge = CvBridge()   # init같은데 미리 CvBridge객체 하나 선언
# cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')    #이미지를 구독했을 때 ros이미지를 opencv 형식으로 변환

# # [반대 version]
# image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")



# load in the calibration data
calib_data_path = "vision/calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 5  # centimeters (measure your printed marker size)

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

param_markers = aruco.DetectorParameters()

cap = cv.VideoCapture(0) # 0, 1 == (/dev/video0),(/dev/video1)


while True:
    ret, frame = cap.read()
    if not ret:
        break
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_left = corners[0].ravel()
            top_right = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()
            middle_right = ((top_right + bottom_right)/2).astype(int)


            # Calculating the distance
            # distance = np.sqrt(
            #     tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            # )
            x = tVec[i][0][0]   # 2th index = form
            y = -tVec[i][0][1]  # change the direction (y-coordinates)
            z = tVec[i][0][2]

            roll = rVec[i][0][0]
            pitch = rVec[i][0][1]
            yaw = rVec[i][0][2]

            print("rVec :", np.round(rVec[i][0], 2))
           

            # Draw the pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            cv.putText(
                frame,
                # f"id: {ids[0]} Dist: {round(distance, 2)}",
                f"id: {ids[0]} ",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255), # BGR
                2,
                cv.LINE_AA,
            )
            cv.putText(
                frame,
                f"x: {round(x, 1)} y: {round(y, 1)} z: {round(z, 1)} ",
                bottom_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (255, 0, 0), # BGR
                2,
                cv.LINE_AA,
            )
            cv.putText(
                frame,
                f"R: {round(roll, 2)} P: {round(pitch, 2)} Y: {round(yaw, 2)}",
                middle_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (100, 255, 150), # BGR
                2,
                cv.LINE_AA,
            )
            # print(ids, "  ", corners)
    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()