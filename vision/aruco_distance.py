# pip install opencv-contrib-python

import cv2 as cv
from cv2 import aruco
import numpy as np

import rclpy as rp
from rclpy.node import Node
from cv_bridge import CvBridge  # [sensor_msgs/msg/Image를 opencv 형식으로 변환]


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
    marker_corners, marker_IDs, reject = aruco.detectMarkers(   # AttributeError 해결 필요
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners is not None and marker_IDs is not None: 
        valid_indices = (marker_IDs.flatten() >= 0) & (marker_IDs.flatten() <= 6) # id 제한(0~6)

        filtered_corners = [marker_corners[i] for i, valid in enumerate(valid_indices) if valid]
        filtered_ids = marker_IDs[valid_indices]
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
            x = tVec[i][0][0]   # 2th index = form
            y = -tVec[i][0][1]  # change the direction (y-coordinates)
            z = tVec[i][0][2]
            distance = np.sqrt((z ** 2) + (x ** 2) + (y ** 2))

            roll = rVec[i][0][0]    # x축회전(roll)
            pitch = rVec[i][0][1]   # z축회전(yaw)
            yaw = rVec[i][0][2]     # y축회전(pitch)

            # Draw the pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            cv.putText(
                frame,
                f"id: {ids[0]} Dist: {round(distance, 2)}",
                # f"ID: {Destination.get(ids[i][0], 'Unknown')}",   # id 제한
                # f"id: {ids[0]} ",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255), # BGR
                2,
                cv.LINE_AA,
            )
            cv.putText(
                frame,
                # f"x: {round(x, 1)} y: {round(y, 1)} z: {round(z, 1)} ",
                f"x: {x:.2f} y: {y:.2f} z: {z:.2f} ",
                bottom_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (255, 0, 0), # BGR
                2,
                cv.LINE_AA,
            )
          
            # print(ids, "  ", corners)

        def aruco_rvec_to_euler_angles(rVec):
            """
            Converts an ArUco marker rotation vector (rVec) to Euler angles (roll, pitch, yaw) in degrees.

            Args:
                rVec (numpy.ndarray): Rotation vector of the ArUco marker (3x1 or 1x3).

            Returns:
                dict: A dictionary with Euler angles in degrees, formatted to 2 decimal places.
                    Keys: "roll", "pitch", "yaw".
            """
            if rVec.shape == (1, 1, 3):
                rVec = rVec.squeeze(axis=0).T   # (3, 1)로 변환

            elif rVec.shape == (2, 1, 3):
                rVec = rVec[0].T

            elif rVec.shape != (3, 1) and rVec.shape != (1, 3):
                raise ValueError(f"Invalid rVec shape: {rVec.shape}.")


            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv.Rodrigues(rVec)

            # Calculate Euler angles from rotation matrix
            sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
            singular = sy < 1e-6

            if not singular:
                roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            else:
                roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                yaw = 0

            # Nomalize roll to be around 10 degrees
            roll_deg = np.degrees(roll)

            if roll_deg > 90:
                roll_deg -= 180
            elif roll_deg < -90:
                roll_deg += 180

            # Convert radians to degrees and format to 2 decimal places
            euler_angles_deg = {
                "roll": float(f"{(-roll_deg):.2f}"),        # z벡터가 위 방향일 때 +
                "pitch": float(f"{np.degrees(pitch):.2f}"), # z벡터가 왼쪽 방향일 때 +
                "yaw": float(f"{np.degrees(yaw):.2f}")      # 시계 방향이 +
            }

            # print("rVec shape:", rVec.shape)
            # print("rVec :", np.round(rVec[i][0], 2))

            return euler_angles_deg

        angles = aruco_rvec_to_euler_angles(rVec)
        # print("Euler Angles (deg):", angles)

        cv.putText(
            frame,
            f"R: {angles['roll']} P: {angles['pitch']} Y: {angles['yaw']} (deg)", 
            middle_right,
            cv.FONT_HERSHEY_PLAIN,
            1.0,
            (150, 255, 50), # BGR
            2,
            cv.LINE_AA,
        )

    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()