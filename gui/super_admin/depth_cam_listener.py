import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge

class DepthCameraVisualization(Node, QWidget):
    def __init__(self, s_admin):
        Node.__init__(self, 'depth_camera_visualization')
        QWidget.__init__(self)

        self.s_admin = s_admin
        self.bridge = CvBridge()
        self.depth_data = None

        # ROS2 Subscriber
        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            1
        )

        # PyQt5 UI setup
        self.setWindowTitle("Depth Camera 3D Visualization")
        self.setGeometry(100, 100, 800, 600)

        # Matplotlib setup
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Depth')

        #뷰 각도 변경
        self.ax.view_init(azim=30, elev=15)

        # Canvas for Matplotlib
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        #self.setLayout(layout)

        #위젯 찾기
        self.view_3d_dep = self.s_admin.findChild(QWidget, "view_3d_dep")
        print(self.view_3d_dep) 
        
        self.view_3d_dep_layout = self.view_3d_dep.layout()
        #self.imu_widget_lidar = LidarVisualization()
        #self.imu_widget_lidar = DepthScanSubscriber(self.s_admin, "/scan")
        self.view_3d_dep_layout.addWidget(self.canvas)


        # Timer for updating the plot
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # Update every 100ms

    def depth_callback(self, msg):
        # Convert ROS2 Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_data = cv_image

    def update_plot(self):
        if self.depth_data is None:
            return

        # Generate 3D points from depth data
        h, w = self.depth_data.shape
        x = np.linspace(-w / 2, w / 2, w)
        y = np.linspace(-h / 2, h / 2, h)
        xv, yv = np.meshgrid(x, y)
        zv = self.depth_data

        self.ax.clear()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Depth')

        # Subsample for faster rendering
        subsample = 10
        self.ax.scatter(xv[::subsample, ::subsample],
                        yv[::subsample, ::subsample],
                        zv[::subsample, ::subsample],
                        c=zv[::subsample, ::subsample], cmap='viridis', s=1)
        
         # Plot red point at the origin
        self.ax.scatter([0], [0], [0], c='red', s=50, label="Origin")
                
        # Add legend
        self.ax.legend()

        # Update plot title and refresh canvas
        self.ax.set_title("3D Camera depth")
        
        self.canvas.draw()

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    s_admin = None

    depth_camera_visualization = DepthCameraVisualization(s_admin)

    # Start PyQt application and ROS2 spinning in parallel
    def ros_spin():
        rclpy.spin(depth_camera_visualization)

    from threading import Thread
    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    depth_camera_visualization.show()
    app.exec_()

    depth_camera_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
