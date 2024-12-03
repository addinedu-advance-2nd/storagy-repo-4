from PyQt5.QtCore import QThread, pyqtSignal, Qt
import cv2
import numpy as np
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget

class CameraThread(QThread):
    frame_ready = pyqtSignal(np.ndarray)  # 프레임 전달 시그널

    def __init__(self, camera_source):
        super().__init__()
        self.camera_source = camera_source  # 카메라 소스 설정
        self.running = False  # 쓰레드 실행 상태

    def run(self):
        self.running = True
        cap = cv2.VideoCapture(self.camera_source)  # 카메라 초기화

        if not cap.isOpened():  # 카메라 연결 실패 시
            print(f"Error: Cannot connect to the camera. Source: {self.camera_source}")
            return

        while self.running:
            ret, frame = cap.read()  # 프레임 읽기
            if ret:
                self.frame_ready.emit(frame)  # 프레임 신호 방출

        cap.release()  # 카메라 해제

    def stop(self):
        self.running = False  # 쓰레드 중지
        self.wait()


class CamStream(QWidget):
    def __init__(self, camera_source, s_admin):
        super().__init__()
        self.s_admin = s_admin

        # UI 설정
        self.rgb_cam = QLabel("카메라 화면")
        self.rgb_cam.setAlignment(Qt.AlignCenter)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.rgb_cam)
        self.setLayout(layout)

        # 카메라 소스 선택
        # 1. 노트북 웹캠 (기본값)
        #camera_source = 0  

        # 2. ESP32-CAM (사용 시 주석 해제)
        # camera_source = "http://192.168.0.100:81/stream"

        # 3. ROS 카메라 토픽 `/camera/color/image_raw` (사용 시 OpenCV로 처리 필요, 주석 해제)
        # camera_source = "/camera/color/image_raw"

        # 카메라 쓰레드 생성
        self.camera_thread = CameraThread(camera_source)

        # 프레임이 준비되면 update_frame 호출
        self.camera_thread.frame_ready.connect(self.update_frame)

        # 쓰레드 시작
        self.camera_thread.start()

    def update_frame(self, frame):
        # 프레임 크기를 320x180으로 조정
        resized_frame = cv2.resize(frame, (320, 240))

        # BGR 이미지를 RGB로 변환
        rgb_image = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        qimg = QImage(rgb_image.data, width, height, width * channel, QImage.Format_RGB888)

        # QLabel에 QPixmap 설정
        self.rgb_cam.setPixmap(QPixmap.fromImage(qimg))
        self.s_admin.rgb_cam.setPixmap(QPixmap.fromImage(qimg))

    def closeEvent(self, event):
        # 창이 닫힐 때 쓰레드 종료
        self.camera_thread.stop()
        event.accept()


# 실행
if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    s_admin = None
    camera_source = 0
    window = CamStream(camera_source, s_admin)
    window.setWindowTitle("멀티 카메라 스트림")
    window.resize(640, 480)
    window.show()
    sys.exit(app.exec_())
