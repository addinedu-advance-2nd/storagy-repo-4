import cv2
import cv2.aruco as aruco
import numpy as np

# 카메라 매트릭스와 왜곡 계수 (캘리브레이션 완료된 값 사용)
# 예제 값: 실제 환경에 맞게 조정 필요
# camera_matrix = np.array([[1000, 0, 640],
#                           [0, 1000, 360],
#                           [0, 0, 1]], dtype=np.float32)
camera_matrix = np.array([[669.33, 0, 640],  # fx
                          [0, 502.0, 360],   # fy
                          [0, 0, 1]], dtype=np.float32)

# dist_coeffs = np.array([0.1, -0.25, 0, 0, 0], dtype=np.float32)
dist_coeffs = np.array([-0.00148085, -0.08140326, -0.00095009,  0.00097647, 0.00895615], dtype=np.float32)

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

# ArUco 마커 탐지 및 각도 계산
def detect_aruco_markers():
    cap = cv2.VideoCapture(0)  # 카메라 열기

    # aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)  # ArUco 사전
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # ArUco 사전
    
    # parameters = aruco.DetectorParameters_create()  # 탐지 파라미터
    parameters = aruco.DetectorParameters()  # 탐지 파라미터
    marker_length = 0.05  # 마커 크기 (미터 단위)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("카메라를 열 수 없습니다.")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 흑백 변환

        # ArUco 마커 탐지
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:

            # 탐지된 마커 그리기
            aruco.drawDetectedMarkers(frame, corners, ids)

            # Pose 추정
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
            for rvec, tvec in zip(rvecs, tvecs):

                # 축 그리기
                # aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                # 회전 벡터 -> 회전 행렬 변환
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # 회전 행렬 -> Euler 각도로 변환
                roll, pitch, yaw = rotation_matrix_to_euler_angles(rotation_matrix)

                # 각도 출력
                print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

                # pitch값 필터링 (일정 범위 이상 값 삭제 band width filter)


        # 결과 출력
        cv2.imshow("ArUco Marker Detection", frame)

        # 종료 키 ('q')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    detect_aruco_markers()