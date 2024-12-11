import sys
import numpy as np
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCostmap  # nav2_msgs.srv 패키지에서 GetCostmap 가져오기
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QGraphicsView, QGraphicsScene
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt

class HeatmapViewer(Node):
    def __init__(self):
        super().__init__('costmap_viewer')

        # /local_costmap/get_costmap 서비스 클라이언트 생성
        self.local_costmap_client = self.create_client(GetCostmap, '/local_costmap/get_costmap')
        self.global_costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')

        # 버튼 생성 및 클릭 이벤트 연결
        self.button_local = QPushButton('Generate Local Costmap')
        self.button_local.setParent(self)  # 부모 위젯 설정
        self.button_local.clicked.connect(self.fetch_local_costmap)

        self.button_global = QPushButton('Generate Global Costmap')
        self.button_global.setParent(self)  # 부모 위젯 설정
        self.button_global.clicked.connect(self.fetch_global_costmap)

        # 버튼 레이아웃 설정
        button_layout = QVBoxLayout()
        button_layout.addWidget(self.button_local)
        button_layout.addWidget(self.button_global)
        
        # 두 개의 뷰 생성
        self.view_local = QGraphicsView(self)
        self.view_global = QGraphicsView(self)

        # 각 뷰에 대해 장면 설정
        self.scene_local = QGraphicsScene()
        self.view_local.setScene(self.scene_local)

        self.scene_global = QGraphicsScene()
        self.view_global.setScene(self.scene_global)

        # 메인 레이아웃 설정
        layout = QHBoxLayout()
        layout.addLayout(button_layout)
        layout.addWidget(self.view_local)
        layout.addWidget(self.view_global)

        self.setLayout(layout)

    def fetch_local_costmap(self):
        if not self.local_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service /local_costmap/get_costmap not available.')
            return
        
        request = GetCostmap.Request()
        future = self.local_costmap_client.call_async(request)
        future.add_done_callback(self.handle_local_costmap_response)

    def fetch_global_costmap(self):
        if not self.global_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service /global_costmap/get_costmap not available.')
            return
        
        request = GetCostmap.Request()
        future = self.global_costmap_client.call_async(request)
        future.add_done_callback(self.handle_global_costmap_response)

    def handle_local_costmap_response(self, future):
        try:
            response = future.result()
            costmap_data = np.array(response.data, dtype=np.uint8).reshape(response.info.height, response.info.width)

            # 히트맵을 위한 데이터 처리
            costmap_array = np.interp(costmap_data, (costmap_data.min(), costmap_data.max()), (0, 255)).astype(np.uint8)

            # QImage로 변환
            image = QImage(costmap_array.data, costmap_array.shape[1], costmap_array.shape[0], costmap_array.shape[1], QImage.Format_Grayscale8)

            # 이미지 확대
            zoomed_image = image.scaled(image.width() * 10, image.height() * 10, Qt.KeepAspectRatio)

            # QPixmap으로 변환
            pixmap = QPixmap(zoomed_image)

            # 로컬 비용맵 뷰에 표시
            self.scene_local.clear()
            self.scene_local.addPixmap(pixmap)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def handle_global_costmap_response(self, future):
        try:
            response = future.result()
            costmap_data = np.array(response.data, dtype=np.uint8).reshape(response.info.height, response.info.width)

            # 히트맵을 위한 데이터 처리
            costmap_array = np.interp(costmap_data, (costmap_data.min(), costmap_data.max()), (0, 255)).astype(np.uint8)

            # QImage로 변환
            image = QImage(costmap_array.data, costmap_array.shape[1], costmap_array.shape[0], costmap_array.shape[1], QImage.Format_Grayscale8)

            # 이미지 확대
            zoomed_image = image.scaled(image.width() * 10, image.height() * 10, Qt.KeepAspectRatio)

            # QPixmap으로 변환
            pixmap = QPixmap(zoomed_image)

            # 글로벌 비용맵 뷰에 표시
            self.scene_global.clear()
            self.scene_global.addPixmap(pixmap)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()

    app = QApplication(sys.argv)
    viewer = HeatmapViewer()

    viewer.show()
    rclpy.spin(viewer)

    app.exec_()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
