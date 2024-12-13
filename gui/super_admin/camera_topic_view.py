import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QHBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from cv_bridge import CvBridge
import cv2 as cv
from cv2 import aruco
import numpy as np

# Load in the calibration data
calib_data_path = "./MultiMatrix.npz"
calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 5  # centimeters (measure your printed marker size)
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters()

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()  # ROS 이미지 메시지를 OpenCV 이미지로 변환
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.image = None  # 이미지를 저장할 변수
        self.image_info = ""  # 토픽에서 수신한 정보 저장

    def image_callback(self, msg):
        try:
            # ROS 메시지에서 OpenCV 이미지로 변환
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 화질(해상도) 조정
            scale_percent = 50  # 크기 비율 (50%로 축소)
            width = int(self.image.shape[1] * scale_percent / 100)
            height = int(self.image.shape[0] * scale_percent / 100)
            dim = (width, height)

            # 이미지 리사이즈
            self.image = cv2.resize(self.image, dim, interpolation=cv2.INTER_AREA)

            # 이미지 정보 업데이트 (축소 후 크기)
            height, width, _ = self.image.shape
            self.image_info = f"Image received and resized: {width}x{height}"

        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image message: {e}")


        # Convert the image to grayscale for ArUco marker detection
        gray_frame = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)

        # Detect ArUco markers
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )

        # If markers are detected
        if marker_corners is not None and marker_IDs is not None:
            # Filter out markers with IDs outside the specified range (0-6)
            valid_indices = (marker_IDs.flatten() >= 0) & (marker_IDs.flatten() <= 6)
            filtered_corners = [marker_corners[i] for i, valid in enumerate(valid_indices) if valid]
            filtered_ids = marker_IDs[valid_indices]

            # Estimate pose for each marker
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                filtered_corners, MARKER_SIZE, cam_mat, dist_coef
            )

            # Iterate over each marker
            for i, (ids, corners) in enumerate(zip(filtered_ids, filtered_corners)):
                # Draw the marker's bounding box
                cv.polylines(
                    self.image, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2).astype(int)
                top_left, top_right, bottom_right, bottom_left = corners
                middle_right = ((top_right + bottom_right) / 2).astype(np.int32)

                # Extract translation and rotation vectors
                x, y, z = tVec[i][0]  # Extract x, y, z from translation vector
                roll, pitch, yaw = rVec[i][0]  # Extract roll, pitch, yaw from rotation vector

                # Draw the axes on the marker
                cv.drawFrameAxes(self.image, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

                # Put the marker's ID and distance on the image
                cv.putText(
                    self.image,
                    f"id: {ids[0]} Dist: {round(np.sqrt(x**2 + y**2 + z**2), 2)}",
                    top_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),  # BGR
                    2,
                    cv.LINE_AA,
                )

                # Put the position information (x, y, z) on the image
                cv.putText(
                    self.image,
                    f"x: {x:.2f} y: {y:.2f} z: {z:.2f}",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (255, 0, 0),  # BGR
                    2,
                    cv.LINE_AA,
                )

        # Display the processed image with detected markers
        cv.imshow("ArUco Marker Detection", self.image)
        key = cv.waitKey(1)
        if key == ord('q'):  # Exit on 'q' key press
            cv.destroyAllWindows()

class VideoWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS Camera Feed")
        self.setGeometry(100, 100, 960, 480)

        # Left side: QLabel to display video
        self.video_label = QLabel(self)
        self.video_label.resize(480, 480)

        # Right side: QLabel to display image info
        self.info_label = QLabel(self)
        self.info_label.resize(480, 480)
        self.info_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        # Layout for the window
        layout = QHBoxLayout()
        layout.addWidget(self.video_label)
        layout.addWidget(self.info_label)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Timer to update the video feed
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_video)
        self.timer.start(100)  # Update every 1s

        # Initialize ROS node
        rclpy.init()
        self.node = CameraNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def update_video(self):
        if self.node.image is not None:
            # Convert OpenCV image (BGR) to RGB
            frame = cv.cvtColor(self.node.image, cv.COLOR_BGR2RGB)

            # Convert the frame to QImage
            h, w, c = frame.shape
            qimg = QImage(frame.data, w, h, w * c, QImage.Format_RGB888)

            # Convert QImage to QPixmap and display
            pixmap = QPixmap.fromImage(qimg)
            self.video_label.setPixmap(pixmap)

            # Update the information on the right side
            self.info_label.setText(self.node.image_info)

    def closeEvent(self, event):
        # Shutdown ROS when closing the application
        self.executor.shutdown()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VideoWindow()
    window.show()

    # Run ROS event loop in a separate thread
    import threading
    ros_thread = threading.Thread(target=rclpy.spin, args=(window.node,))
    ros_thread.start()

    sys.exit(app.exec_())
