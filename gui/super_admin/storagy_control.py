import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton

# 1. RobotMover 클래스 (ROS 2 노드)
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


# 2. JoyTeleop 클래스 (조이스틱 제어를 위한 ROS 2 노드)
class JoyTeleop(Node):
    def __init__(self, robot_mover):
        super().__init__('joy_teleop')
        self.robot_mover = robot_mover
        self.joy_subscriber = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        # 왼쪽 스틱은 전진/후진 제어, 오른쪽 스틱은 회전 제어
        linear_x = msg.axes[1]  # 왼쪽 스틱 Y축 (전진/후진)
        angular_z = msg.axes[0]  # 오른쪽 스틱 X축 (회전)
        
        # 속도 제한을 위해 0.5와 1로 조정
        linear_x = linear_x * 0.5
        angular_z = angular_z * 1.0

        # 로봇 제어 명령을 보내기
        self.robot_mover.send_command(linear_x, angular_z)


# 3. StoragyControl 클래스 (PyQt5 GUI)
class StoragyControl(QMainWindow):
    def __init__(self, s_admin, state):
        super().__init__()
        self.s_admin = s_admin  # UI 파일 참조
        self.setWindowTitle("Robot Controller")

        # 제어 방법 설정 (keyboard 또는 joystick)
        self.control_state = state

        self.robot_mover = RobotMover()  # 로봇 제어 객체 생성
        self.joy_teleop = JoyTeleop(self.robot_mover)  # 조이스틱 제어 객체 생성

        self.setup_ui()

    def setup_ui(self):
        self.setGeometry(100, 100, 400, 300)

        # UI에서 버튼들을 가져와 연결
        self.forward_button = self.s_admin.findChild(QPushButton, 'forward_button')
        self.backward_button = self.s_admin.findChild(QPushButton, 'backward_button')
        self.left_button = self.s_admin.findChild(QPushButton, 'left_button')
        self.right_button = self.s_admin.findChild(QPushButton, 'right_button')
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
        if self.control_state == 'keyboard':
            print("키보드로 전진")
            self.robot_mover.send_command(linear_x=0.5)
        elif self.control_state == 'joystick':
            self.joy_teleop.joy_callback(Joy())  # 실제로 조이스틱 입력을 처리해야 함

    def move_backward(self):
        if self.control_state == 'keyboard':
            print("키보드로 후진")
            self.robot_mover.send_command(linear_x=-0.5)
        elif self.control_state == 'joystick':
            self.joy_teleop.joy_callback(Joy())  # 실제로 조이스틱 입력을 처리해야 함

    def turn_left(self):
        if self.control_state == 'keyboard':
            print("키보드로 왼쪽 회전")
            self.robot_mover.send_command(angular_z=0.5)
        elif self.control_state == 'joystick':
            self.joy_teleop.joy_callback(Joy())  # 실제로 조이스틱 입력을 처리해야 함

    def turn_right(self):
        if self.control_state == 'keyboard':
            print("키보드로 오른쪽 회전")
            self.robot_mover.send_command(angular_z=-0.5)
        elif self.control_state == 'joystick':
            self.joy_teleop.joy_callback(Joy())  # 실제로 조이스틱 입력을 처리해야 함

    def stop(self):
        if self.control_state == 'keyboard':
            print("키보드로 정지")
            self.robot_mover.send_command()
        elif self.control_state == 'joystick':
            self.joy_teleop.joy_callback(Joy())  # 실제로 조이스틱 입력을 처리해야 함

    def rotate_left(self):
        if self.control_state == 'keyboard':
            print("키보드로 왼쪽 회전")
            self.robot_mover.send_command(angular_z=1.0)
        elif self.control_state == 'joystick':
            self.joy_teleop.joy_callback(Joy())  # 실제로 조이스틱 입력을 처리해야 함

    def rotate_right(self):
        if self.control_state == 'keyboard':
            print("키보드로 오른쪽 회전")
            self.robot_mover.send_command(angular_z=-1.0)
        elif self.control_state == 'joystick':
            self.joy_teleop.joy_callback(Joy())  # 실제로 조이스틱 입력을 처리해야 함


# 메인 함수
def main():
    rclpy.init()  # ROS 2 초기화

    # PyQt5 애플리케이션 생성
    app = QApplication([])

    # 실제 UI 객체를 여기에 전달해야 합니다.
    s_admin = None  # 실제 s_admin 객체를 여기에 대입해야 합니다.
    
    # 제어 방식 설정 (keyboard 또는 joystick)
    control_state = 'keyboard'  # 예: 'keyboard' 또는 'joystick'
    window = StoragyControl(s_admin, control_state)
    window.show()

    # PyQt5 이벤트 루프 실행
    app.exec_()

    # 종료 시 ROS 2 종료
    rclpy.shutdown()


if __name__ == '__main__':
    main()
