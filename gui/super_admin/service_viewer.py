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

    # ROS 2 서비스 호출 메서드
    def call_service(self, service_name, request):
        # 클라이언트 생성
        client = self.create_client(type(request), service_name)
        # 서비스가 사용 가능한지 확인
        if not client.wait_for_service(timeout_sec=5.0):
            return f"Service {service_name} not available."
        # 비동기 호출 및 결과 처리
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return str(future.result())
        else:
            return f"Service call to {service_name} failed."

    # 특정 서비스 이름에 해당하는 타입 가져오기
    def get_service_types(self, service_name):
        services = self.get_service_names_and_types()
        # 서비스를 딕셔너리로 변환
        service_dict = {name: types for name, types in services}
        # 주어진 서비스 이름의 타입 반환
        return service_dict.get(service_name, [])


class ServiceViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 Service Caller")
        self.setGeometry(100, 100, 800, 600)

        # 스타일 시트 적용
        self.setStyleSheet("""
            QLabel, QListWidget {
                color: white;  /* 글자 색상 흰색 */
                background-color: #2c2d30;  /* 배경 색상 검정 */
            }
            QTextEdit, QLineEdit {
                color: white;  /* 글자 색상 흰색 */
                background-color: #2c2d30;  /* 배경 색상 검정 */
            }
            QPushButton {
                color: white;
                background-color: #2c2d30;  /* 버튼 배경 회색 */
            }
        """)

        # 레이아웃 생성
        main_layout = QHBoxLayout()  # 메인 레이아웃: 수평
        left_layout = QVBoxLayout()  # 왼쪽 레이아웃: 수직
        right_layout = QVBoxLayout()  # 오른쪽 레이아웃: 수직

        # 왼쪽 패널: 서비스 목록과 입력 필드
        self.service_list = QListWidget()  # 서비스 리스트 위젯
        self.input_field = QLineEdit()  # 파라미터 입력 필드
        self.call_button = QPushButton("Call Service")  # 서비스 호출 버튼

        # 왼쪽 패널에 위젯 추가
        left_layout.addWidget(QLabel("서비스 리스트:"))
        left_layout.addWidget(self.service_list)
        left_layout.addWidget(QLabel("파라미터 입력:"))
        left_layout.addWidget(self.input_field)
        left_layout.addWidget(self.call_button)

        # 오른쪽 패널: 결과 표시
        self.result_display = QTextEdit()  # 결과 표시용 텍스트 에디터
        self.result_display.setReadOnly(True)  # 읽기 전용으로 설정

        right_layout.addWidget(QLabel("수신된 데이터:"))
        right_layout.addWidget(self.result_display)

        # 레이아웃 결합
        main_layout.addLayout(left_layout, 2)  # 왼쪽 패널 추가 (비율 2)
        main_layout.addLayout(right_layout, 5)  # 오른쪽 패널 추가 (비율 5)
        self.setLayout(main_layout)

        # ROS 2 노드 생성
        self.ros_node = ROS2ServiceCaller()

        # 타이머로 서비스 목록 업데이트
        self.service_update_timer = QTimer(self)
        self.service_update_timer.timeout.connect(self.update_service_list)
        self.service_update_timer.start(5000)  # 5초마다 업데이트

        # 버튼 및 이벤트 연결
        self.call_button.clicked.connect(self.call_service)  # 호출 버튼 클릭 시 실행
        self.service_list.itemSelectionChanged.connect(self.update_input_field_state)  # 선택 변경 시 실행

    # 서비스 목록 업데이트
    def update_service_list(self):
        try:
            # `ros2 service list` 명령 실행하여 서비스 목록 가져오기
            result = subprocess.run(['ros2', 'service', 'list'], capture_output=True, text=True, check=True)
            services = result.stdout.strip().split('\n')  # 결과를 줄바꿈으로 분리
            self.service_list.clear()  # 기존 목록 삭제
            self.service_list.addItems(services)  # 새로운 서비스 추가
        except subprocess.CalledProcessError as e:
            self.result_display.append(f"[Error] Failed to fetch services: {e}")

    # 입력 필드 활성화/비활성화
    def update_input_field_state(self):
        selected_service = self.service_list.currentItem()  # 선택된 서비스 가져오기
        if not selected_service:
            self.input_field.setDisabled(True)  # 선택된 서비스가 없으면 비활성화
            return

        service_name = selected_service.text()
        service_types = self.ros_node.get_service_types(service_name)

        # 입력 필드 활성화 여부 결정 (예: 'Trigger'가 포함되지 않은 경우 활성화)
        if service_types and 'Trigger' not in service_types[0]:
            self.input_field.setDisabled(False)
        else:
            self.input_field.setDisabled(True)

    # 서비스 호출
    def call_service(self):
        selected_service = self.service_list.currentItem()  # 선택된 서비스 가져오기
        if not selected_service:
            self.result_display.append("[Error] No service selected.")
            return

        service_name = selected_service.text()
        input_params = self.input_field.text()

        # 서비스 요청 객체 생성 (예: Trigger)
        from example_interfaces.srv import Trigger
        request = Trigger.Request()  # 올바른 요청 객체로 변경 필요

        try:
            result = self.ros_node.call_service(service_name, request)
            self.result_display.append(f"[Success] {result}")
        except Exception as e:
            self.result_display.append(f"[Error] {str(e)}")

    # 창 닫힐 때 ROS 2 종료
    def closeEvent(self, event):
        rclpy.shutdown()
        super().closeEvent(event)

# 메인 실행
if __name__ == '__main__':
    rclpy.init()  # ROS 2 초기화
    app = QApplication(sys.argv)
    window = ServiceViewer()
    window.show()
    sys.exit(app.exec_())
