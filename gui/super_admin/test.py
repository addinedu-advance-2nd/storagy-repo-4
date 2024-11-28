# main.py
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QFrame
from view_3D import IMUVisualization  # 기존 코드 파일을 `imu_visualization.py`로 저장했다고 가정

class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("IMU Visualization Viewer")
        self.setGeometry(200, 200, 800, 600)

        # 중앙 위젯 설정
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        # 레이아웃 설정
        layout = QVBoxLayout(central_widget)

        # QFrame으로 컨테이너 추가
        imu_frame = QFrame(self)
        imu_frame.setFrameShape(QFrame.Box)  # 테두리 설정
        imu_frame.setLineWidth(2)
        imu_frame_layout = QVBoxLayout(imu_frame)

        # IMUVisualization 위젯 추가
        self.imu_widget = IMUVisualization()
        imu_frame_layout.addWidget(self.imu_widget)

        # IMU Frame을 메인 레이아웃에 추가
        layout.addWidget(imu_frame)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainApp()
    main_window.show()
    sys.exit(app.exec_())
