import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 예시로 String 메시지 사용
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel

class Ros2Subscriber(Node):
    def __init__(self, label):
        super().__init__('ros2_subscriber_node')
        self.subscription = self.create_subscription(
            String,  # 메시지 타입
            'battery_voltage',  # 구독할 토픽 이름
            self.listener_callback,
            10  # 큐 사이즈
        )
        self.label = label

    def listener_callback(self, msg):
        # ROS 2 토픽에서 받은 메시지를 PyQt UI에 업데이트
        self.label.setText(f'Received Message: {msg.data}')

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS2 Topic Display')
        self.setGeometry(100, 100, 400, 200)
        layout = QVBoxLayout()
        
        self.label = QLabel('Waiting for message...')
        layout.addWidget(self.label)
        
        self.setLayout(layout)

        # ROS 2 구독자 초기화
        self.ros2_subscriber = Ros2Subscriber(self.label)

def main(args=None):
    rclpy.init(args=args)

    # PyQt 애플리케이션 초기화
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # ROS 2 노드 실행
    rclpy.spin(window.ros2_subscriber)

    app.exec_()

if __name__ == '__main__':
    main()
