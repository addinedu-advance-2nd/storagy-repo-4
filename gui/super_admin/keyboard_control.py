import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton
from PyQt5.QtCore import Qt, QThread
import sys

# ROS 2 노드 (RobotMover)
class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Robot Mover Node Initialized')

    def send_command(self, linear_x=0.0, angular_z=0.0):
        move_cmd = Twist()
        move_cmd.linear.x = linear_x
        move_cmd.angular.z = angular_z
        self.publisher_.publish(move_cmd)
        self.get_logger().info(f'Publishing: linear.x={linear_x}, angular.z={angular_z}')


# ROS 2 스핀을 실행할 스레드
class RclpyThread(QThread):
    def run(self):
        rclpy.spin(self.node)  # ROS 2 spin을 별도의 스레드에서 실행

    def __init__(self, node):
        super().__init__()
        self.node = node


# PyQt GUI (KeyBoardControl)
class KeyBoardControl(QMainWindow):
    def __init__(self, s_admin):
        super().__init__()
        self.s_admin = s_admin
        self.setWindowTitle("Robot Controller")

        self.robot_mover = RobotMover()  # ROS 2 노드 초기화
        self.setup_ui()

        # 포커스를 받을 수 있도록 설정
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()  # 윈도우가 열리면 포커스 설정

        # ROS 2 스핀을 위한 스레드 시작
        self.rclpy_thread = RclpyThread(self.robot_mover)
        self.rclpy_thread.start()

        

    def setup_ui(self):
        self.setGeometry(100, 100, 400, 300)

        # UI에서 버튼들을 가져와 연결
        self.forward_button = self.s_admin.findChild(QPushButton, 'move_forward_button')
        self.backward_button = self.s_admin.findChild(QPushButton, 'move_backward_button')
        self.left_button = self.s_admin.findChild(QPushButton, 'turn_left_button')
        self.right_button = self.s_admin.findChild(QPushButton, 'turn_right_button')
        self.stop_button = self.s_admin.findChild(QPushButton, 'stop_button')
        self.rotate_left_button = self.s_admin.findChild(QPushButton, 'rotate_left_button')
        self.rotate_right_button = self.s_admin.findChild(QPushButton, 'rotate_right_button')

        # 버튼 클릭 이벤트 연결
        self.forward_button.clicked.connect(self.move_forward)
        self.backward_button.clicked.connect(self.move_backward)
        self.left_button.clicked.connect(self.turn_left)
        self.right_button.clicked.connect(self.turn_right)
        self.stop_button.clicked.connect(self.stop)
        self.rotate_left_button.clicked.connect(self.rotate_left)
        self.rotate_right_button.clicked.connect(self.rotate_right)

    def move_forward(self):
        print("전진")
        self.robot_mover.send_command(linear_x=0.5)

    def move_backward(self):
        print("후진")
        self.robot_mover.send_command(linear_x=-0.5)

    def turn_left(self):
        print("좌회전")
        self.robot_mover.send_command(linear_x=0.5, angular_z=0.5)

    def turn_right(self):
        print("우회전")
        self.robot_mover.send_command(linear_x=0.5, angular_z=-0.5)

    def stop(self):
        print("정지")
        self.robot_mover.send_command()

    def rotate_left(self):
        print("왼쪽 회전")
        self.robot_mover.send_command(angular_z=1.0)

    def rotate_right(self):
        print("오른쪽 회전")
        self.robot_mover.send_command(angular_z=-1.0)

    def keyPressEvent(self, event):
        key = event.key()
        print(f"Pressed key: {key}")

        # 키보드 버튼에 따라 매핑된 버튼 클릭
        if key == Qt.Key_1:
            self.move_forward()
        elif key == Qt.Key_S:
            self.move_backward()
        elif key == Qt.Key_A:
            self.turn_left()
        elif key == Qt.Key_D:
            self.turn_right()
        elif key == Qt.Key_Space:
            self.stop()
        elif key == Qt.Key_Q:
            self.rotate_left()
        elif key == Qt.Key_E:
            self.rotate_right()
        else:
            super().keyPressEvent(event)


# 메인 함수
def main():
    rclpy.init()  # ROS 2 초기화

    # PyQt5 애플리케이션 생성
    app = QApplication([])

    # UI 객체를 전달해야 합니다. 실제 UI 객체를 사용하세요.
    #s_admin = None  # 실제 s_admin 객체를 여기에 전달

    # 제어 방식 설정 (keyboard)
    window = KeyBoardControl()
    window.show()

    # PyQt5 이벤트 루프 실행
    app.exec_()

    # 종료 시 ROS 2 종료
    rclpy.shutdown()


if __name__ == '__main__':
    main()
