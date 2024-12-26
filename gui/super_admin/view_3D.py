import sys
import numpy as np
import random
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from mpl_toolkits.mplot3d import Axes3D
from threading import Thread

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

class IMUVisualization(Node, QWidget):
    def __init__(self, node_name):
        Node.__init__(self, node_name)
        QWidget.__init__(self)

        self.x_data = []
        self.y_data = []
        self.z_data = []

        self.setWindowTitle(f'{node_name} 3D Visualization')
        self.setGeometry(100, 100, 800, 600)

        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.canvas = FigureCanvasQTAgg(self.figure)
        layout = QVBoxLayout(self)
        layout.addWidget(self.canvas)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

        if node_name == 'depth_camera':
            self.create_subscription(
                Image,
                '/camera/depth/image_raw',
                self.depth_callback,
                10
            )
        elif node_name == 'lidar':
            self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                10
            )

    def depth_callback(self, msg):
        x, y, z = self.process_depth_data(msg)
        self.add_data(x, y, z)

    def scan_callback(self, msg):
        x, y, z = self.process_scan_data(msg)
        self.add_data(x, y, z)

    def process_depth_data(self, msg):
        return random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)

    def process_scan_data(self, msg):
        return random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)

    def update_plot(self):
        self.ax.clear()
        self.ax.scatter(self.x_data, self.y_data, self.z_data, c='b', s=50)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.canvas.draw()

    def add_data(self, x, y, z):
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)


def ros_spin(node):
    rclpy.spin(node)


def main():
    app = QApplication([])
    rclpy.init()

    node_name = 'lidar'  # 또는 'depth_camera'
    imu_visualization = IMUVisualization(node_name)

    ros_thread = Thread(target=ros_spin, args=(imu_visualization,), daemon=True)
    ros_thread.start()

    app.exec_()

    imu_visualization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
