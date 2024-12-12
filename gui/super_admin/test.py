import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from cv2 import aruco
import numpy as np

# Load in the calibration data
calib_data_path = "aruco_distance_estimation/calib_data/MultiMatrix.npz"
calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 5  # centimeters (measure your printed marker size)
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters()

class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        
        # Create a CvBridge object to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Subscriber for the camera image
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Adjust the topic name if necessary
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Convert the ROS image message to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting ROS image to OpenCV format: {e}")
            return

        # Convert the image to grayscale for ArUco marker detection
        gray_frame = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

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
                    cv_image, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2).astype(int)
                top_left, top_right, bottom_right, bottom_left = corners
                middle_right = ((top_right + bottom_right) / 2).astype(np.int32)

                # Extract translation and rotation vectors
                x, y, z = tVec[i][0]  # Extract x, y, z from translation vector
                roll, pitch, yaw = rVec[i][0]  # Extract roll, pitch, yaw from rotation vector

                # Draw the axes on the marker
                cv.drawFrameAxes(cv_image, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

                # Put the marker's ID and distance on the image
                cv.putText(
                    cv_image,
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
                    cv_image,
                    f"x: {x:.2f} y: {y:.2f} z: {z:.2f}",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (255, 0, 0),  # BGR
                    2,
                    cv.LINE_AA,
                )

        # Display the processed image with detected markers
        cv.imshow("ArUco Marker Detection", cv_image)
        key = cv.waitKey(1)
        if key == ord('q'):  # Exit on 'q' key press
            cv.destroyAllWindows()

def main():
    rclpy.init()
    aruco_marker_detector = ArucoMarkerDetector()

    # Spin the ROS 2 node to keep it running and processing the images
    rclpy.spin(aruco_marker_detector)

    # Clean up and shutdown
    aruco_marker_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
