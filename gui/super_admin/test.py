import sys
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QPushButton, QWidget
from PyQt5.QtCore import QObject, QThread, pyqtSignal
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


# ROS Node Wrapper with PyQt QObject
class MapSubscriberNode(QObject):
    map_received = pyqtSignal(object)  # PyQt 시그널

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.ros_node = Node('map_subscriber_node')
        self.subscription = self.ros_node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.ros_thread = QThread()
        self.moveToThread(self.ros_thread)
        self.ros_thread.started.connect(self.spin_ros_node)
        self.ros_thread.start()

    def map_callback(self, msg):
        self.map_received.emit(msg)  # PyQt 시그널로 메시지 전달

    def spin_ros_node(self):
        try:
            rclpy.spin(self.ros_node)  # ROS 2 이벤트 루프 실행
        except Exception as e:
            print(f"Error in ROS spin: {e}")

    def stop(self):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.ros_thread.quit()
        self.ros_thread.wait()


# PyQt5 MainWindow
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("ROS 2 Map Subscriber with Heatmap")
        self.resize(800, 600)

        self.label = QLabel("Waiting for map data...", self)
        self.show_heatmap_btn = QPushButton("Show Heatmap", self)
        self.show_heatmap_btn.setEnabled(False)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.show_heatmap_btn)
        self.setLayout(layout)

        # ROS Node 초기화
        self.ros_node = MapSubscriberNode()
        self.ros_node.map_received.connect(self.handle_map_data)

        # 버튼 이벤트
        self.show_heatmap_btn.clicked.connect(self.show_heatmap)
        

        self.current_map_data = None

    def handle_map_data(self, msg):
        self.label.setText("Map data received!")
        self.current_map_data = msg  # 메시지 저장
        self.show_heatmap_btn.setEnabled(True)

    def show_heatmap(self):
        if self.current_map_data is not None:
            self.visualize_map(self.current_map_data)

    def visualize_map(self, msg):
        width = msg.info.width
        height = msg.info.height

        # OccupancyGrid 데이터를 2D 배열로 변환
        map_data = np.array(msg.data).reshape((height, width))

        # 히트맵 시각화
        plt.figure(figsize=(10, 8))
        plt.imshow(map_data, cmap='gray', origin='lower')
        plt.colorbar(label='Cell Value')
        plt.title('Map Heatmap')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.show()

    def closeEvent(self, event):
        # 종료 시 ROS 노드 정리
        self.ros_node.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
