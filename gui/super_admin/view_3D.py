# matplotlib 설치 : pip install matplotlib

import sys
import numpy as np
import random
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer, pyqtSignal
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D

class IMUVisualization(QWidget):
    # 텍스트 업데이트 시그널 정의
    update_rotation_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        self.setWindowTitle('IMU 3D Visualization')
        self.setGeometry(100, 100, 800, 600)
        
        # Matplotlib 캔버스를 위한 컨테이너 위젯 설정
        self.layout = QVBoxLayout(self)
        
        # Matplotlib Figure와 3D 축 설정
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection='3d')
        
        # PyQt에서 Matplotlib 캔버스를 표시
        self.canvas = FigureCanvas(self.figure)
        self.layout.addWidget(self.canvas)
        
        # 초기 좌표 및 그래프 구성
        self.x_data = []
        self.y_data = []
        self.z_data = []
        
        # 플롯의 축 설정
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_zlim(-10, 10)
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        
        # 그래프 업데이트를 위한 타이머 설정
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

        # 회전 값 표시 레이블
        self.rotation_label = QLabel(self)
        self.rotation_label.setText(f"Rotation (x, y, z): (0, 0, 0)")
        self.layout.addWidget(self.rotation_label)


        # 큐브의 좌표 (3배 크기 조정)
        self.cube_x = [-2, 2, 2, -2, -2, 2, 2, -2]
        self.cube_y = [-2, -2, 2, 2, -2, -2, 2, 2]
        self.cube_z = [-2, -2, -2, -2, 2, 2, 2, 2]

        
        # 큐브의 엣지 정의 (점들을 연결)
        self.cube_edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7)
        ]

        # 회전 매트릭스 초기화
        self.rotation_matrix = np.eye(3)

        # 키보드 입력 값 초기화
        self.azim = -25
        self.elev = 20

    def update_plot(self):
        # IMU 데이터 업데이트 (임시로 랜덤 값 사용)
        x, y, z = self.get_imu_data()
        
        # 새로운 데이터 추가
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)
        
        # 현재 플롯을 지우고 새로운 데이터를 플로팅
        self.ax.cla()  # 축 초기화
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(0, 20)
        self.ax.set_zlim(0, 20)
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        
        # 새로운 IMU 데이터를 빨간 점으로 표시
        #self.ax.scatter(self.x_data, self.y_data, self.z_data, color='r', s=50)

        # 큐브 그리기 (회전 적용)
        self.draw_cube()
        
        # 회전 값 레이블 업데이트
        self.update_rotation_label()
        
        # 시그널을 보내서 다른 파일에 전달
        self.update_rotation_signal.emit(f"Rotation (x, y, z): ({self.elev}, {self.azim}, 0)")
        
        # 캔버스 다시 그리기
        self.canvas.draw()

    def get_imu_data(self):
        # 랜덤 데이터로 IMU 값 생성 (실제 센서 데이터로 대체 필요)
        x, y, z = random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)
        return x, y, z

    def draw_cube(self):
        # 큐브의 각 엣지 연결하여 그리기
        for start, end in self.cube_edges:
            # 큐브의 각 점에 회전 적용
            x_start, y_start, z_start = self.rotate_point(self.cube_x[start], self.cube_y[start], self.cube_z[start])
            x_end, y_end, z_end = self.rotate_point(self.cube_x[end], self.cube_y[end], self.cube_z[end])
            self.ax.plot([x_start, x_end], [y_start, y_end], [z_start, z_end], color='b')

    def rotate_point(self, x, y, z):
        # 3D 회전 매트릭스를 사용하여 점을 회전
        point = np.array([x, y, z])
        rotated_point = np.dot(self.rotation_matrix, point)
        return rotated_point[0], rotated_point[1], rotated_point[2]

    def update_rotation_label(self):
        # 회전 값 레이블을 갱신
        self.rotation_label.setText(f"Rotation (x, y, z): ({self.elev}, {self.azim}, 0)")

    def keyPressEvent(self, event):
        # 키보드로 카메라 회전 조정
        if event.key() == 16777234:  # 왼쪽 화살표 (방위각 감소)
            self.azim -= 5
        elif event.key() == 16777236:  # 오른쪽 화살표 (방위각 증가)
            self.azim += 5
        elif event.key() == 16777235:  # 위쪽 화살표 (엘리베이션 증가)
            self.elev += 5
        elif event.key() == 16777237:  # 아래쪽 화살표 (엘리베이션 감소)
            self.elev -= 5
        
        # 회전 매트릭스를 업데이트하여 전체 좌표계 회전
        self.update_rotation_matrix()
        
        # 회전 값 레이블 업데이트
        self.update_rotation_label()
        
        # 캔버스 다시 그리기
        self.canvas.draw()

    def update_rotation_matrix(self):
        # 회전 매트릭스를 업데이트
        elev_rad = np.radians(self.elev)
        azim_rad = np.radians(self.azim)
        
        # 엘리베이션 회전 (X축)
        elev_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(elev_rad), -np.sin(elev_rad)],
            [0, np.sin(elev_rad), np.cos(elev_rad)]
        ])
        
        # 방위각 회전 (Y축)
        azim_matrix = np.array([
            [np.cos(azim_rad), 0, np.sin(azim_rad)],
            [0, 1, 0],
            [-np.sin(azim_rad), 0, np.cos(azim_rad)]
        ])
        
        # 최종 회전 매트릭스 계산 (순서대로 회전)
        self.rotation_matrix = np.dot(azim_matrix, elev_matrix)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = IMUVisualization()
    window.show()
    sys.exit(app.exec_())
