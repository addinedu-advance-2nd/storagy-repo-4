from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QLabel, QVBoxLayout, QGridLayout, QGroupBox
)
from PyQt5.QtCore import Qt, QTimer, QThread
from PyQt5.QtGui import QImage, QPixmap
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
import numpy as np

class VisualizationApp(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        # Window setup
        self.setWindowTitle("ROS 2 Navigation Visualization")
        self.setGeometry(100, 100, 800, 600)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QGridLayout()
        self.central_widget.setLayout(self.layout)

        # Create and add group boxes for each ROS data section
        self.map_group = self.create_group_box("Map", QLabel("No map data yet"))
        self.path_group = self.create_group_box("Path", QLabel("No path data yet"))
        self.odom_group = self.create_group_box("Odometry", QLabel("No odometry data yet"))
        self.goal_group = self.create_group_box("Goal Pose", QLabel("No goal data yet"))
        self.local_costmap_group = self.create_group_box("Local Costmap", QLabel("No local costmap data yet"))
        self.global_costmap_group = self.create_group_box("Global Costmap", QLabel("No global costmap data yet"))
        self.scan_group = self.create_group_box("Scan", QLabel("No scan data yet"))
        self.pointcloud_group = self.create_group_box("Point Cloud", QLabel("No point cloud data yet"))

        # Add all sections to the grid layout (2 rows x 4 columns)
        self.layout.addWidget(self.map_group, 0, 0)
        self.layout.addWidget(self.path_group, 0, 1)
        self.layout.addWidget(self.odom_group, 0, 2)
        self.layout.addWidget(self.goal_group, 0, 3)
        self.layout.addWidget(self.local_costmap_group, 1, 0)
        self.layout.addWidget(self.global_costmap_group, 1, 1)
        self.layout.addWidget(self.scan_group, 1, 2)
        self.layout.addWidget(self.pointcloud_group, 1, 3)

        # Start periodic UI updates
        self.update_ui()

    def create_group_box(self, title, content_label):
        """
        Creates a group box containing a title and a label for content display.
        """
        group_box = QGroupBox(title)
        layout = QVBoxLayout()
        content_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(content_label)
        group_box.setLayout(layout)
        return group_box

    def update_ui(self):
        """
        Updates the user interface with the latest ROS data.
        """
        # Update Map visualization
        if self.ros_node.map_data is not None:
            self.map_group.layout().itemAt(0).widget().setText("Map data updated")
        else:
            self.map_group.layout().itemAt(0).widget().setText("Waiting for map data...")

        # Update Path visualization
        if self.ros_node.path_data is not None:
            self.path_group.layout().itemAt(0).widget().setText(f"Path received: {len(self.ros_node.path_data)} points")
        else:
            self.path_group.layout().itemAt(0).widget().setText("Waiting for path data...")

        # Update Odometry visualization
        if self.ros_node.odom_data is not None:
            odom = self.ros_node.odom_data
            self.odom_group.layout().itemAt(0).widget().setText(f"Odometry: x={odom[0]:.2f}, y={odom[1]:.2f}")
        else:
            self.odom_group.layout().itemAt(0).widget().setText("Waiting for odometry data...")

        # Update Goal Pose visualization
        if self.ros_node.goal_pose is not None:
            goal = self.ros_node.goal_pose
            self.goal_group.layout().itemAt(0).widget().setText(f"Goal: x={goal[0]:.2f}, y={goal[1]:.2f}")
        else:
            self.goal_group.layout().itemAt(0).widget().setText("Waiting for goal pose...")

        # Update Local Costmap visualization
        if self.ros_node.local_costmap_data is not None:
            heatmap_image = self.create_heatmap(self.ros_node.local_costmap_data)
            self.local_costmap_group.layout().itemAt(0).widget().setPixmap(QPixmap.fromImage(heatmap_image))
        else:
            self.local_costmap_group.layout().itemAt(0).widget().setText("Waiting for local costmap data...")

        # Update Global Costmap visualization
        if self.ros_node.global_costmap_data is not None:
            heatmap_image = self.create_heatmap(self.ros_node.global_costmap_data)
            self.global_costmap_group.layout().itemAt(0).widget().setPixmap(QPixmap.fromImage(heatmap_image))
        else:
            self.global_costmap_group.layout().itemAt(0).widget().setText("Waiting for global costmap data...")

        # Update Scan visualization
        if self.ros_node.scan_data is not None:
            self.scan_group.layout().itemAt(0).widget().setText(f"Scan data: {len(self.ros_node.scan_data)} ranges")
        else:
            self.scan_group.layout().itemAt(0).widget().setText("Waiting for scan data...")

        # Update Point Cloud visualization
        if self.ros_node.pointcloud_data is not None:
            self.pointcloud_group.layout().itemAt(0).widget().setText("Point cloud data received")
        else:
            self.pointcloud_group.layout().itemAt(0).widget().setText("Waiting for point cloud data...")

        # Refresh periodically
        QTimer.singleShot(100, self.update_ui)

    def create_heatmap(self, data):
        """
        Creates a heatmap QImage from a 2D array of data.
        """
        fig, ax = plt.subplots()
        canvas = FigureCanvas(fig)
        ax.imshow(data, cmap='hot', interpolation='nearest')
        ax.axis('off')

        canvas.draw()
        width, height = canvas.get_width_height()
        image = QImage(canvas.buffer_rgba(), width, height, QImage.Format_ARGB32)
        return image

class ROS2Thread(QThread):
    """
    ROS2Thread extends QThread for managing ROS 2 communication within the PyQt application.
    """
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

    def run(self):
        """
        The thread's run logic. Replace 'spin()' with appropriate ROS 2 node spinning logic.
        """
        self.ros_node.spin()

if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication
    import random

    class DummyROSNode:
        """ Dummy ROS Node for testing the UI without ROS 2 dependencies. """
        def __init__(self):
            self.map_data = None
            self.path_data = None
            self.odom_data = None
            self.goal_pose = None
            self.local_costmap_data = None
            self.global_costmap_data = None
            self.scan_data = None
            self.pointcloud_data = None

        def spin(self):
            pass  # Replace with actual ROS 2 spin logic

        def generate_dummy_data(self):
            self.map_data = True
            self.path_data = [random.random() for _ in range(10)]
            self.odom_data = (random.uniform(-10, 10), random.uniform(-10, 10))
            self.goal_pose = (random.uniform(-10, 10), random.uniform(-10, 10))
            self.local_costmap_data = np.random.rand(10, 10)
            self.global_costmap_data = np.random.rand(20, 20)
            self.scan_data = [random.uniform(0, 10) for _ in range(360)]
            self.pointcloud_data = True

    app = QApplication(sys.argv)
    ros_node = DummyROSNode()

    # Generate dummy data periodically
    def update_dummy_data():
        ros_node.generate_dummy_data()
        QTimer.singleShot(1000, update_dummy_data)

    update_dummy_data()

    window = VisualizationApp(ros_node)
    window.show()
    sys.exit(app.exec_())
