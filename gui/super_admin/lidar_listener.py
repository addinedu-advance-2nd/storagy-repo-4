import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge

class LidarVisualization(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'lidar_visualization')
        QWidget.__init__(self)

        self.bridge = CvBridge()
        self.lidar_data = None  # Initialize lidar_data

        # ROS2 Subscriber
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            1
        )

        # PyQt5 UI setup
        self.setWindowTitle("Lidar 3D Visualization")
        self.setGeometry(100, 100, 800, 600)

        # Matplotlib setup
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Depth')

        # Set view angle
        self.ax.view_init(azim=30, elev=15)

        # Canvas for Matplotlib
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # Timer for updating the plot
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(500)  # Update every 500ms

    def lidar_callback(self, msg):
        # Process the LaserScan data
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Calculate x, y, z from polar coordinates
        x = -ranges * np.cos(angles)  # Reverse X for mirroring
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)  # 2D scan, so z = 0

        # Store lidar data
        self.lidar_data = np.array([x, y, z])

    def update_plot(self):
        if self.lidar_data is None:
            print("No lidar data available")
            return

        # Extract x, y, z from lidar data
        x, y, z = self.lidar_data

        if len(x) != len(y) or len(y) != len(z):
            print("Data dimension mismatch: x, y, z must have the same length")
            return

        # Clear previous plot
        self.ax.clear()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # Calculate distances for coloring
        distance = np.sqrt(x**2 + y**2 + z**2)

        # Apply subsampling for performance
        subsample = 1
        self.ax.scatter(x[::subsample], y[::subsample], z[::subsample],
                        c=distance[::subsample], cmap='viridis', s=0.1)

        # Plot red point at the origin
        self.ax.scatter([0], [0], [0], c='red', s=50, label="Origin")

            # Fix axis limits for consistent visualization
        self.ax.set_xlim(-1, 1)  # Adjust based on your Lidar range
        self.ax.set_ylim(-1, 1)  # Adjust based on your Lidar range
        self.ax.set_zlim(-0.5, 0.5)    # Adjust for Z if necessary

        # Add legend
        self.ax.legend()

        # Update plot title and refresh canvas
        self.ax.set_title("Lidar Scan")
        self.canvas.draw()






def main():
    rclpy.init()
    app = QApplication(sys.argv)

    lidar_visualization = LidarVisualization()

    # Start PyQt application and ROS2 spinning in parallel
    def ros_spin():
        rclpy.spin(lidar_visualization)

    from threading import Thread
    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    lidar_visualization.show()
    app.exec_()

    lidar_visualization.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

