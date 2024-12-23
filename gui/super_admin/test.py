import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class ScanSubscriber(Node):
    def __init__(self, label, ax):
        super().__init__('scan_subscriber')
        self.label = label
        self.ax = ax
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)

    def scan_callback(self, msg):
        # Convert the ranges to Cartesian coordinates
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Calculate x and y from polar coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)  # 2D scan, so z = 0

        # Clear the plot and plot the new scan data
        self.ax.cla()  # Clear the axes
        self.ax.scatter(x, y, z, c=z, cmap='viridis', marker='o')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title("3D Laser Scan Visualization")

        # Redraw the plot
        self.ax.figure.canvas.draw()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Laser Scan Data 3D Visualization")
        self.setGeometry(100, 100, 800, 600)

        # Set up the layout and the central widget
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        layout = QVBoxLayout(self.central_widget)

        # Create a Matplotlib figure and 3D axis
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection='3d')

        # Create a canvas for embedding the plot in the PyQt window
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        # Initialize ROS and the scan subscriber
        rclpy.init()
        self.scan_subscriber = ScanSubscriber(self.central_widget, self.ax)

        self.show()

    def closeEvent(self, event):
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
