import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton

# RobotMover 클래스 (ROS 2 노드)
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


# KeyBoardControl 클래스 (PyQt5 GUI, keyboard control)
class KeyBoardControl(QMainWindow):
    def __init__(self, s_admin):
        super().__init__()
        self.s_admin = s_admin  # UI 파일 참조
        self.setWindowTitle("Robot Controller")

        # 제어 방법 설정 (keyboard)
        #self.control_state = state

        self.robot_mover = RobotMover()  # 로봇 제어 객체 생성

        self.setup_ui()

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
        print("키보드로 전진")
        self.robot_mover.send_command(linear_x=0.5)

    def move_backward(self):
        print("키보드로 후진")
        self.robot_mover.send_command(linear_x=-0.5)

    def turn_left(self):
        print("키보드로 좌회전")
        self.robot_mover.send_command(linear_x=0.5, angular_z=0.5)

    def turn_right(self):
        print("키보드로 우회전")
        self.robot_mover.send_command(linear_x=0.5, angular_z=-0.5)

    def stop(self):
        print("키보드로 정지")
        self.robot_mover.send_command()

    def rotate_left(self):
        print("키보드로 왼쪽 회전")
        self.robot_mover.send_command(angular_z=1.0)

    def rotate_right(self):
        print("키보드로 오른쪽 회전")
        self.robot_mover.send_command(angular_z=-1.0)


# 메인 함수
def main():
    rclpy.init()  # ROS 2 초기화

    # PyQt5 애플리케이션 생성
    app = QApplication([])

    # 실제 UI 객체를 여기에 전달해야 합니다.
    s_admin = None  # 실제 s_admin 객체를 여기에 대입해야 합니다.
    
    # 제어 방식 설정 (keyboard)
    control_state = 'keyboard'
    window = KeyBoardControl(s_admin)
    window.show()

    # PyQt5 이벤트 루프 실행
    app.exec_()

    # 종료 시 ROS 2 종료
    rclpy.shutdown()


if __name__ == '__main__':
    main()
