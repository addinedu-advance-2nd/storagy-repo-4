import sys
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLineEdit, QTextEdit, QListWidget, QLabel
)
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node

class ROS2ServiceCaller(Node):
    def __init__(self):
        super().__init__('service_caller')

    def call_service(self, service_name, request):
        client = self.create_client(type(request), service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            return f"Service {service_name} not available."
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return str(future.result())
        else:
            return f"Service call to {service_name} failed."

    def get_service_types(self, service_name):
        services = self.get_service_names_and_types()  # 리스트 반환
        # 리스트를 딕셔너리로 변환
        service_dict = {name: types for name, types in services}
        # 주어진 서비스 이름의 타입을 반환
        return service_dict.get(service_name, [])



class ServiceViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 Service Caller")
        self.setGeometry(100, 100, 800, 600)

        # Layouts
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        # Left panel: Service list and input
        self.service_list = QListWidget()
        self.input_field = QLineEdit()
        self.call_button = QPushButton("Call Service")

        left_layout.addWidget(QLabel("서비스 리스트:"))
        left_layout.addWidget(self.service_list)
        left_layout.addWidget(QLabel("파라미터 입력:"))
        left_layout.addWidget(self.input_field)
        left_layout.addWidget(self.call_button)

        # Right panel: Result display
        self.result_display = QTextEdit()
        self.result_display.setReadOnly(True)

        right_layout.addWidget(QLabel("수신된 데이터:"))
        right_layout.addWidget(self.result_display)

        # Combine layouts
        main_layout.addLayout(left_layout, 2)
        main_layout.addLayout(right_layout, 5)
        self.setLayout(main_layout)

        # ROS 2 node

        self.ros_node = ROS2ServiceCaller()

        # Timer to update service list
        self.service_update_timer = QTimer(self)
        self.service_update_timer.timeout.connect(self.update_service_list)
        self.service_update_timer.start(5000)  # Update every 5 seconds

        # Connections
        self.call_button.clicked.connect(self.call_service)
        self.service_list.itemSelectionChanged.connect(self.update_input_field_state)

    def update_service_list(self):
        try:
            # Execute `ros2 service list` to get available services
            result = subprocess.run(['ros2', 'service', 'list'], capture_output=True, text=True, check=True)
            services = result.stdout.strip().split('\n')
            self.service_list.clear()
            self.service_list.addItems(services)
        except subprocess.CalledProcessError as e:
            self.result_display.append(f"[Error] Failed to fetch services: {e}")

    def update_input_field_state(self):
        selected_service = self.service_list.currentItem()
        if not selected_service:
            self.input_field.setDisabled(True)
            return

        service_name = selected_service.text()
        service_types = self.ros_node.get_service_types(service_name)

        # Enable input field only if the service requires input
        if service_types and 'Trigger' not in service_types[0]:  # Example logic
            self.input_field.setDisabled(False)
        else:
            self.input_field.setDisabled(True)

    def call_service(self):
        selected_service = self.service_list.currentItem()
        if not selected_service:
            self.result_display.append("[Error] No service selected.")
            return

        service_name = selected_service.text()
        input_params = self.input_field.text()

        # Here, you'd create a proper request object based on the service type
        # For example purposes, we'll send a mock request:
        from example_interfaces.srv import Trigger
        request = Trigger.Request()  # Replace with the correct request type

        try:
            result = self.ros_node.call_service(service_name, request)
            self.result_display.append(f"[Success] {result}")
        except Exception as e:
            self.result_display.append(f"[Error] {str(e)}")

    def closeEvent(self, event):
        rclpy.shutdown()
        super().closeEvent(event)

if __name__ == '__main__':
    rclpy.init()
    app = QApplication(sys.argv)
    window = ServiceViewer()
    window.show()
    sys.exit(app.exec_())
