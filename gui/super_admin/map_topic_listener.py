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
        self.ros_thread = QThread()
        self.moveToThread(self.ros_thread)

        # ROS 노드 초기화
        self.ros_node = None
        self.ros_thread.started.connect(self.init_ros)
        self.ros_thread.start()

    def init_ros(self):
        rclpy.init()
        self.ros_node = Node('map_subscriber_node')
        self.subscription = self.ros_node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            1
        )
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

    def map_callback(self, msg):
        self.map_received.emit(msg)  # PyQt 시그널로 전달

    def stop(self):
        if self.ros_node:
            self.ros_node.destroy_node()
        rclpy.shutdown()
        self.ros_thread.quit()
        self.ros_thread.wait()


class MapTopicListener(QWidget):
    def __init__(self, s_admin):
        super().__init__()
        self.s_admin = s_admin
        self.init_ui()

        # ROS Node 초기화
        self.ros_node = MapSubscriberNode()
        self.ros_node.map_received.connect(self.handle_map_data)

        self.current_map_data = None

        # s_admin.map_widget에 UI 추가
        if hasattr(self.s_admin, 'map_widget'):
            self.s_admin.map_widget.setLayout(QVBoxLayout())
            self.s_admin.map_widget.layout().addWidget(self)

    def init_ui(self):
        self.setWindowTitle("ROS 2 Map Subscriber with Heatmap")

        self.label = QLabel("Waiting for map data...", self)
        self.show_heatmap_btn = QPushButton("Show Heatmap", self)
        self.show_heatmap_btn.setEnabled(False)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.show_heatmap_btn)
        self.setLayout(layout)

        self.show_heatmap_btn.clicked.connect(self.show_heatmap)

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
        map_data = np.array(msg.data).reshape((height, width))

        # 히트맵 표시
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

    # s_admin 객체 생성
    s_admin = QWidget()
    s_admin.map_widget = QWidget()

    s_admin_layout = QVBoxLayout(s_admin)
    s_admin_layout.addWidget(s_admin.map_widget)

    map_listener = MapTopicListener(s_admin)
    s_admin.show()

    sys.exit(app.exec_())



