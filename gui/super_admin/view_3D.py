# matplotlib 설치 : pip install matplotlib

import sys
import numpy as np
import random
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer, pyqtSignal
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from mpl_toolkits.mplot3d import Axes3D

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan




# IMUVisualization 클래스 (x, y, z 데이터를 3D로 그리기)
class IMUVisualization(QWidget):
    def __init__(self, node_name):
        super().__init__()

        # 노드 이름에 따라 시각화할 데이터를 구분
        self.node_name = node_name

        # x, y, z 데이터를 저장할 리스트
        self.x_data = []
        self.y_data = []
        self.z_data = []

        # UI 설정
        self.setWindowTitle(f'{node_name} 3D Visualization')
        self.setGeometry(100, 100, 800, 600)

        # Matplotlib 캔버스 생성 (3D 플롯)
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # Matplotlib 캔버스를 PyQt 위젯에 삽입
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout = QVBoxLayout(self)
        layout.addWidget(self.canvas)

        # 타이머 설정 (100ms마다 plot을 업데이트)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(1)

        # 노드 이름에 따라 구독할 토픽을 설정
        if node_name == 'depth_camera':
            self.depth_subscriber = self.create_subscription(
                Image,
                '/camera/depth/image_raw',
                self.depth_callback,
                10)
            self.depth_subscriber
        elif node_name == 'lidar':
            self.scan_subscriber = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                10)
            self.scan_subscriber

    def depth_callback(self, msg):
        # 깊이 데이터 처리 (예시로 임의의 x, y, z 값을 전달)
        x, y, z = self.process_depth_data(msg)
        # IMUVisualization에 데이터 추가
        self.add_data(x, y, z)

    def scan_callback(self, msg):
        # LiDAR 데이터 처리 (예시로 임의의 x, y, z 값을 전달)
        x, y, z = self.process_scan_data(msg)
        # IMUVisualization에 데이터 추가
        self.add_data(x, y, z)

    def process_depth_data(self, msg):
        # 깊이 데이터를 처리하여 x, y, z 값을 추출하는 부분 (예시)
        x = 1  # 실제 x 데이터 처리
        y = 1  # 실제 y 데이터 처리
        z = 1  # 실제 z 데이터 처리
        return x, y, z

    def process_scan_data(self, msg):
        # LiDAR 데이터를 처리하여 x, y, z 값을 추출하는 부분 (예시)
        x = 0  # 실제 x 데이터 처리
        y = 0  # 실제 y 데이터 처리
        z = 0  # 실제 z 데이터 처리
        return x, y, z

    def update_plot(self):
        # 데이터를 갱신하여 3D 플롯에 그리기
        self.ax.clear()  # 기존 플롯을 지우고

        self.x_data, self.y_data, self.z_data = self.get_imu_data()
        # x, y, z = self.get_imu_data()
                # Append new data to the lists
        #self.x_data.append(x)
        #self.y_data.append(y)
        #self.z_data.append(z)

        #self.x_data = x
        #self.y_data = y
        #self.z_data = z

        # node_name에 따라 다른 데이터를 시각화
        if self.node_name == 'depth_camera':
            # 뎁스 카메라 데이터를 3D로 시각화
            self.ax.scatter(self.x_data, self.y_data, self.z_data, c='g', s=50)
        elif self.node_name == 'lidar':
            # LiDAR 데이터를 3D로 시각화
            self.ax.scatter(self.x_data, self.y_data, self.z_data, c='r', s=50)
        else:
            # test
            self.ax.scatter(self.x_data, self.y_data, self.z_data, c='b', s=1)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # 화면에 캔버스 그리기
        self.canvas.draw()

    def add_data(self, x, y, z):
        # SensorSubscriber에서 호출되는 데이터 추가 함수
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

    def get_imu_data(self):
        # Simulate IMU data. Replace this with actual sensor readings.
        # For demonstration, generate random data within a range
        #x = random.uniform(-10, 10)
        #y = random.uniform(-10, 10)
        #z = random.uniform(-10, 10)
        x_data=[]
        y_data=[]
        z_data=[]
        i=0
        for i in range(0,501):
            x = random.uniform(-10, 10)
            y = random.uniform(7, 9)
            z = random.uniform(-10, 10)
            x_data.append(x)
            y_data.append(y)
            z_data.append(z)
            

        return x_data, y_data, z_data



# 메인 함수 (PyQt 애플리케이션과 ROS 2 실행)
def main():
    # PyQt 애플리케이션 생성
    app = QApplication([])

    # node_name에 따라 IMUVisualization 객체 생성
    #node_name = 'lidar'  # 또는 'depth_camera'
    #imu_visualization = IMUVisualization(node_name)

    # SensorSubscriber 객체 생성
    #sensor_subscriber = SensorSubscriber(imu_visualization, node_name)

    # ROS 2 실행
    rclpy.spin(IMUVisualization)

    # PyQt 애플리케이션 실행
    app.exec_()


if __name__ == '__main__':
    main()


