import sys
import time
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QScrollArea, QHBoxLayout, QPushButton, QSplitter
)
from PyQt5.QtCore import QThread, Qt, pyqtSignal, QTimer
import importlib


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


def format_message(message):
    try:
        return str(message)
    except Exception as e:
        return f"Error formatting message: {e}"


class ROS2SubscriberNode(Node):
    def __init__(self):
        super().__init__('ros2_ui_subscriber')
        self.topic_list = []
        self.subscribers = {}
        self.topic_messages = {}

    def update_topics(self):
        self.topic_list = self.get_topic_names_and_types()

    def subscribe_to_topic(self, topic_name, message_type):
        if topic_name not in self.subscribers:
            msg_class = get_message_class(message_type)
            if not msg_class:
                self.get_logger().error(f"Failed to load message type for topic: {topic_name}")
                return

            self.subscribers[topic_name] = self.create_subscription(
                msg_class,
                topic_name,
                lambda msg, t=topic_name: self.topic_callback(t, msg),
                10
            )

    def topic_callback(self, topic_name, message):
        formatted_message = format_message(message)
        self.topic_messages[topic_name] = formatted_message


class ROS2Worker(QThread):
    topic_signal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()

        self.node = ROS2SubscriberNode()
        self.running = True

    def run(self):
        start_time = time.time()
        while time.time() - start_time < 10 and self.running:
            rclpy.spin_once(self.node, timeout_sec=1)
            self.node.update_topics()
            topics = {t[0]: t[1][0] for t in self.node.topic_list}
            self.topic_signal.emit(topics)

        while self.running:
            rclpy.spin_once(self.node, timeout_sec=1)

    def stop(self):
        self.running = False
        rclpy.shutdown()


class TopicView(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 Topics Viewer")

        self.splitter = QSplitter(Qt.Horizontal)
        self.setCentralWidget(self.splitter)

        self.left_widget = QWidget()
        self.left_layout = QVBoxLayout()
        self.left_widget.setLayout(self.left_layout)

        # 왼쪽 위젯의 최소 폭 설정
        self.left_widget.setMinimumWidth(300)  # 좌측 창의 최소 폭을 300px로 설정
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidget(self.left_widget)
        self.scroll_area.setWidgetResizable(True)
        self.splitter.addWidget(self.scroll_area)

        self.right_widget = QWidget()
        self.right_layout = QVBoxLayout()
        self.right_widget.setLayout(self.right_layout)
        self.message_label = QLabel("Select a topic to view its messages.")
        self.message_label.setWordWrap(True)
        self.right_layout.addWidget(self.message_label)
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidget(self.right_widget)
        self.scroll_area.setWidgetResizable(True)

        self.splitter.addWidget(self.scroll_area)

        # QSplitter의 비율을 설정하여 왼쪽과 오른쪽 창의 시작 비율을 지정
        self.splitter.setSizes([300, 1100])  # 왼쪽 300px, 오른쪽 600px로 시작

        self.ros2_worker = ROS2Worker()
        self.ros2_worker.topic_signal.connect(self.update_topics)
        self.ros2_worker.start()

        self.topic_buttons = {}
        self.current_topic = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_message)
        self.timer.start(1000)  # 1초 간격으로 메시지 업데이트

    # 나머지 코드는 그대로 유지됩니다.


    def update_topics(self, topics):
        for topic, msg_type in topics.items():
            if topic not in self.topic_buttons:
                topic_layout = QHBoxLayout()
                label = QLabel(f"Topic : {topic}")
                button = QPushButton("View")
                button.setFixedSize(80, 30)  # 고정된 버튼 크기
                button.clicked.connect(self.create_button_handler(topic))
                topic_layout.addWidget(label)
                topic_layout.addWidget(button)

                topic_widget = QWidget()
                topic_widget.setLayout(topic_layout)
                self.left_layout.addWidget(topic_widget)

                self.topic_buttons[topic] = button
                self.ros2_worker.node.subscribe_to_topic(topic, msg_type)

    def create_button_handler(self, topic_name):
        def handler():
            self.current_topic = topic_name
            self.message_label.setText("Loading messages...")
        return handler

    def update_message(self):
        if self.current_topic:
            message = self.ros2_worker.node.topic_messages.get(self.current_topic, "No message received yet.")
            self.display_message(message)

    def display_message(self, message):
        self.message_label.setText(message)

    def closeEvent(self, event):
        self.ros2_worker.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    rclpy.init()
    app = QApplication(sys.argv)
    window = TopicView()
    window.show()
    sys.exit(app.exec_())
