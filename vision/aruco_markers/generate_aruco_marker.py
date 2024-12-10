#pip install opencv-contrib-python


import cv2
import cv2.aruco as aruco

# 마커를 생성할 딕셔너리 선택 (여기서는 4x4_50 사용)
# aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# 생성할 마커 ID와 크기 설정
marker_id = 49  # 마커 ID (0 ~ 49)
marker_size = 1000  # 마커 크기 (n*n 픽셀)

# 빈 이미지를 생성하고 아루코 마커를 그리기
# marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size)
marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

print(cv2.__version__)

# 마커를 이미지로 저장
output_filename = f"aruco_marker_{marker_id}.png"
cv2.imwrite(output_filename, marker_image)

print(f"ArUco 마커가 '{output_filename}' 이름으로 저장되었습니다.")