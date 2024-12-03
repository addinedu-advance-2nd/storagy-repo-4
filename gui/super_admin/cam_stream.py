# cam_stream.py

from PyQt5.QtCore import QThread, pyqtSignal, Qt
import cv2
import numpy as np
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QLabel

class CameraThread(QThread):
    frame_ready = pyqtSignal(np.ndarray)  # 프레임 전달 시그널

    def __init__(self, camera_source):
        super().__init__()
        self.camera_source = camera_source
        self.running = False

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(self.camera_source)

        if not cap.isOpened():
            print("Error: Cannot connect to the camera.")
            return

        while self.running:
            ret, frame = cap.read()
            if ret:
                self.frame_ready.emit(frame)

        cap.release()

    def stop(self):
        self.running = False
        self.wait()

class CamStream:
    def __init__(self, s_admin):
        self.s_admin = s_admin

        # 카메라 소스 설정
        #camera_source = "http://192.168.0.100:81/stream"  # ESP32-CAM IP
        camera_source = 0  # 테스트 시 노트북 웹캠 사용

        # 카메라 쓰레드 생성
        self.camera_thread = CameraThread(camera_source)
        
        # 프레임이 준비되면 update_frame 메서드 호출
        self.camera_thread.frame_ready.connect(self.update_frame)

        # 쓰레드 시작
        self.camera_thread.start()

    def update_frame(self, frame):
        #320*180으로 리사이즈
        resized_frame = cv2.resize(frame, (320, 180))

        # OpenCV 이미지를 QImage로 변환
        rgb_image = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        qimg = QImage(rgb_image.data, width, height, width * channel, QImage.Format_RGB888)

        # QLabel 크기에 맞게 프레임 크기 조정
        #height, width = self.s_admin.rgb_cam.height(), self.s_admin.rgb_cam.width()
        #resized_frame = cv2.resize(frame, (width, height))

        # UI의 QLabel에 QImage를 설정
        self.s_admin.rgb_cam.setPixmap(QPixmap.fromImage(qimg))
        self.s_admin.rgb_cam.setAlignment(Qt.AlignCenter)



# 실행
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CamStream()
    window.show()
    sys.exit(app.exec_())
