import sys
import subprocess
import importlib
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QTextEdit, QListWidget, QLabel
)
from PyQt5.QtCore import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 예시로 String 메시지 사용



class ROS2TopicHandler(Node):
    def __init__(self):
        super().__init__('topic_handler')

    def get_topic_types(self):
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, check=True)
        topics = result.stdout.strip().split('\n')
        return topics

    def get_topic_type(self, topic_name):
        result = subprocess.run(['ros2', 'topic', 'type', topic_name], capture_output=True, text=True, check=True)
        return result.stdout.strip()

def get_message_class(message_type):
    try:
        parts = message_type.split('/')
        if len(parts) < 2:
            raise ValueError("Invalid message type format")

        package = parts[0]
        msg = parts[-1]
        module = importlib.import_module(f'{package}.msg')
        return getattr(module, msg)
    except Exception as e:
        print(f"Error importing message type {message_type}: {e}")
        return None

class TopicViewer(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 Topic Subscriber")
        self.setGeometry(100, 100, 800, 600)

        # 레이아웃
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        # 왼쪽 패널: 토픽 리스트와 구독 버튼
        self.topic_list = QListWidget()
        self.subscribe_button = QPushButton("구독 시작")

        left_layout.addWidget(QLabel("토픽 리스트:"))
        left_layout.addWidget(self.topic_list)
        left_layout.addWidget(self.subscribe_button)

        # 오른쪽 패널: 수신된 데이터 표시
        self.result_display = QTextEdit()
        self.result_display.setReadOnly(True)

        right_layout.addWidget(QLabel("수신된 데이터:"))
        right_layout.addWidget(self.result_display)

        # 레이아웃 결합
        main_layout.addLayout(left_layout, 2)
        main_layout.addLayout(right_layout, 5)
        self.setLayout(main_layout)

        # ROS 2 노드
        self.ros_node = ROS2TopicHandler()

        # 100초마다 토픽 리스트 갱신
        self.topic_update_timer = QTimer(self)
        self.topic_update_timer.timeout.connect(self.update_topic_list)
        self.topic_update_timer.start(1000)  # 100초마다 업데이트

        # 구독 버튼 클릭 시 토픽 구독 시작
        self.subscribe_button.clicked.connect(self.start_subscribing)

        # ROS2Subscriber를 위한 워커 스레드 생성
        self.ros_worker = ROS2SubscriberWorker(self.ros_node)
        self.ros_worker.message_signal.connect(self.update_received_message)
        self.ros_worker.start()

    def update_topic_list(self):
        try:
            # 사용 가능한 토픽을 리스트에 추가
            topics = self.ros_node.get_topic_types()
            topics.sort()  # 토픽 이름 알파벳 순으로 정렬
            self.topic_list.clear()
            self.topic_list.addItems(topics)
        except subprocess.CalledProcessError as e:
            self.result_display.append(f"[Error] Failed to fetch topics: {e}")

    def start_subscribing(self):
        selected_topic = self.topic_list.currentItem()
        if not selected_topic:
            self.result_display.append("[Error] No topic selected.")
            return

        topic_name = selected_topic.text()
        self.result_display.append(f"[Info] Subscribing to topic: {topic_name}")

        # 선택된 토픽의 메시지 타입 가져오기
        topic_type = self.ros_node.get_topic_type(topic_name)
        self.result_display.append(f"[Info] Topic type: {topic_type}")

        # 해당 메시지 타입에 맞게 구독 시작
        self.subscribe_to_topic(topic_name, topic_type)

    def subscribe_to_topic(self, topic_name, topic_type):
        # 동적으로 메시지 클래스를 가져오기
        msg_class = get_message_class(topic_type)
        print(msg_class)

        if msg_class:
            # 토픽 구독 시작
            self.ros_node.create_subscription(
                msg_class,
                topic_name,
                self.ros_worker.topic_callback,
                10
            )
            print(f"[Info] Successfully subscribed to topic: {topic_name}")
        else:
            self.result_display.append(f"[Error] Unsupported message type for topic: {topic_name}")
            print(f"[Error] Unsupported message type for topic: {topic_name}")

    def update_received_message(self, msg):
        # QTextEdit에서 기존 내용 지우기
        self.result_display.clear()

        # 수신된 메시지를 오른쪽 화면에 출력
        self.result_display.append(f"Received: {msg.data}")
        print(f"Received: {msg.data}")

    def closeEvent(self, event):
        rclpy.shutdown()
        super().closeEvent(event)

class ROS2SubscriberWorker(QThread):
    message_signal = pyqtSignal(String)  # 메시지를 UI로 전달

    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

    def topic_callback(self, msg):
        # 수신된 메시지를 UI로 전달
        self.message_signal.emit(msg)

if __name__ == '__main__':
    rclpy.init()
    app = QApplication(sys.argv)
    window = TopicViewer()
    window.show()
    sys.exit(app.exec_())
