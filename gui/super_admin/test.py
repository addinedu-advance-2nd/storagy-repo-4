import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton, QVBoxLayout, QWidget

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


class JoyTeleop(Node):
    def __init__(self, robot_mover):
        super().__init__('joy_teleop')
        self.robot_mover = robot_mover
        self.joy_subscriber = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        # 왼쪽 스틱을 전진/후진 제어, 오른쪽 스틱을 회전 제어
        linear_x = msg.axes[1]  # 왼쪽 스틱 Y축 (전진/후진)
        angular_z = msg.axes[0]  # 오른쪽 스틱 X축 (회전)
        
        # 0.5와 -0.5 값으로 조정 (속도 제한)
        linear_x = linear_x * 0.5
        angular_z = angular_z * 1.0

        # 로봇 제어 명령 보내기
        self.robot_mover.send_command(linear_x, angular_z)

class MainWindow(QMainWindow):
    def __init__(self, s_admin):
        super().__init__()
        self.s_admin = s_admin  # UI 파일 참조
        self.setWindowTitle("Robot Controller")

        self.robot_mover = RobotMover()  # 로봇 제어 객체 생성
        self.joy_teleop = JoyTeleop(self.robot_mover)  # 조이스틱 제어 객체 생성
        self.setup_ui()

    def setup_ui(self):
        self.setGeometry(100, 100, 400, 300)

        # 버튼들 정의
        self.forward_button = QPushButton('Forward', self)
        self.backward_button = QPushButton('Backward', self)
        self.left_button = QPushButton('Turn Left', self)
        self.right_button = QPushButton('Turn Right', self)
        self.stop_button = QPushButton('Stop', self)
        self.rotate_left_button = QPushButton('Rotate Left', self)
        self.rotate_right_button = QPushButton('Rotate Right', self)

        # 버튼 클릭 이벤트 연결
        self.forward_button.clicked.connect(self.move_forward)
        self.backward_button.clicked.connect(self.move_backward)
        self.left_button.clicked.connect(self.turn_left)
        self.right_button.clicked.connect(self.turn_right)
        self.stop_button.clicked.connect(self.stop)
        self.rotate_left_button.clicked.connect(self.rotate_left)
        self.rotate_right_button.clicked.connect(self.rotate_right)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.forward_button)
        layout.addWidget(self.backward_button)
        layout.addWidget(self.left_button)
        layout.addWidget(self.right_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.rotate_left_button)
        layout.addWidget(self.rotate_right_button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def move_forward(self):
        self.robot_mover.send_command(linear_x=0.5)

    def move_backward(self):
        self.robot_mover.send_command(linear_x=-0.5)

    def turn_left(self):
        self.robot_mover.send_command(angular_z=0.5)

    def turn_right(self):
        self.robot_mover.send_command(angular_z=-0.5)

    def stop(self):
        self.robot_mover.send_command()

    def rotate_left(self):
        self.robot_mover.send_command(angular_z=1.0)

    def rotate_right(self):
        self.robot_mover.send_command(angular_z=-1.0)

def main():
    rclpy.init()  # ROS 2 초기화

    # PyQt5 애플리케이션 생성
    app = QApplication([])

    # s_admin 객체를 전달하여 UI 생성 (UI 파일에서 참조된 객체)
    s_admin = None  # 실제 s_admin 객체를 여기에 대입해야 합니다.
    window = MainWindow(s_admin)
    window.show()

    # PyQt5 이벤트 루프 실행
    app.exec_()

    # 종료 시 ROS 2 종료
    rclpy.shutdown()

if __name__ == '__main__':
    main()
