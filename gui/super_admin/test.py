import numpy as np
import cv2

# Euler 각도 변환 함수
def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.degrees(x), np.degrees(y), np.degrees(z)

# 주어진 rVec 값들
rvecs = [
    [2.94, 0.21, -0.43], [2.96, 0.19, -0.54], [2.96, 0.19, -0.54],
    [2.92, 0.19, -0.56], [2.95, 0.18, -0.52], [2.95, 0.18, -0.52],
    [2.93, 0.19, -0.62], [2.95, 0.18, -0.52], [2.95, 0.19, -0.47],
    [2.96, 0.19, -0.54], [2.96, 0.19, -0.54], [2.96, 0.19, -0.54],
    [2.96, 0.19, -0.54], [2.96, 0.19, -0.54], [2.96, 0.19, -0.54],
    [2.96, 0.19, -0.54], [2.84, 0.13, -0.67], [2.7, 0.05, -0.74],
    [2.52, -0.1, -0.76], [2.31, -0.25, -0.68]
]

# 각도 변환 수행
for rvec in rvecs:
    rvec = np.array(rvec, dtype=np.float32)
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)
    
    print(f"rVec: {rvec}")
    print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
    print("-" * 30)
