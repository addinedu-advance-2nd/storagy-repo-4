import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
import numpy as np
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class DepthScanSubscriber(Node):
    def __init__(self, s_admin, topic_name):
        super().__init__('depth_scan_subscriber')
        self.s_admin = s_admin
        self.topic_name = topic_name

        if self.topic_name == "/camera/depth/image_raw" :
            # Depth 이미지와 LaserScan 메시지 구독
            self.depth_subscription = self.create_subscription(
                Image,
                '/camera/depth/image_raw',
                self.depth_image_callback,
                10
            )
            
        elif self.topic_name == "/scan":
            self.scan_subscription = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                10
            )

        self.bridge = CvBridge()
        
        # Matplotlib 3D 화면 설정
        self.fig = plt.figure(figsize=(12, 6))
        self.ax_depth = self.fig.add_subplot(121, projection='3d')
        self.ax_scan = self.fig.add_subplot(122, projection='3d')
        plt.ion()  # Interactive mode 시작
        plt.show()

    def depth_image_callback(self, msg):
        # ROS 메시지 -> OpenCV 이미지로 변환
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Ensure depth_image is 2D
        if depth_image.ndim == 1:
            depth_image = depth_image.reshape((1, -1))  # Reshape if it's a 1D array

        # Depth 이미지를 3D로 표시
        height, width = depth_image.shape
        X, Y = np.meshgrid(np.arange(width), np.arange(height))
        Z = depth_image  # Z is the depth image itself

        # Transparency mask based on Z values (make z=0 transparent)
        alpha = np.where(Z > 0, 1.0, 0.0)  # Set alpha to 1.0 where Z > 0, else 0.0

        # 3D 그래프 그리기
        self.ax_depth.cla()  # 이전 내용 지우기
        scatter = self.ax_depth.scatter(
            X - width // 2, Y - height // 2, Z, c=Z, cmap='viridis', s=0.5, alpha=alpha.flatten()
        )
        self.ax_depth.set_title('3D Depth Image')
        self.ax_depth.set_xlabel('X (pixel)')
        self.ax_depth.set_ylabel('Y (pixel)')
        self.ax_depth.set_zlabel('Z (depth)')

        # Z축 범위 조정
        self.ax_depth.set_zlim(np.min(Z[Z > 0]), np.max(Z))  # Exclude Z=0 from range

        # 화면 업데이트
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.01)  # 화면을 갱신하도록 잠시 대기



    def scan_callback(self, msg):
        # LaserScan 데이터를 polar 좌표로 변환
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        
        # 라이다 데이터를 3D 좌표로 변환 (원점을 (0, 0, 0)으로 맞추기 위해 변환)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(ranges)  # Z값은 0으로 설정 (2D 라이다이기 때문에)

        # 라이다의 원점을 (0, 0, 0)으로 맞추기 위한 변환
        lidar_center_x = 0
        lidar_center_y = 0
        lidar_center_z = 0

        # 3D 그래프 그리기
        self.ax_scan.cla()  # 이전 내용 지우기
        scatter = self.ax_scan.scatter(x, y, z, c=ranges, cmap='jet', s=1)
        self.ax_scan.set_title('3D LaserScan Data')
        self.ax_scan.set_xlabel('X (m)')
        self.ax_scan.set_ylabel('Y (m)')
        self.ax_scan.set_zlabel('Z (m)')

        # 라이다 중심점 표시
        self.ax_scan.scatter(lidar_center_x, lidar_center_y, lidar_center_z, color='red', s=100, label='Lidar Center')

        # Z축 범위 조정
        self.ax_scan.set_zlim(np.min(z), np.max(z))

        # X, Y축 범위 조정 (범위가 너무 크면 스케일을 맞추기 위해 약간의 여유를 둡니다)
        self.ax_scan.set_xlim(np.min(x) - 1, np.max(x) + 1)
        self.ax_scan.set_ylim(np.min(y) - 1, np.max(y) + 1)

        # 화면 업데이트
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.01)  # 화면을 갱신하도록 잠시 대기

def main(args=None):
    rclpy.init(args=args)
    s_admin = None
    node_name = "/scan"
    depth_scan_subscriber = DepthScanSubscriber(s_admin, node_name)
    
    try:
        rclpy.spin(depth_scan_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        depth_scan_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()