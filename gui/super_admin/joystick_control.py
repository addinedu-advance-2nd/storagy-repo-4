

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QMainWindow, QApplication
import inputs  # 조이스틱 입력 라이브러리

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


# JoyStickControl 클래스 (PyQt5 GUI, 조이스틱 제어)
class JoyStickControl(QMainWindow):
    def __init__(self, s_admin):
        super().__init__()
        self.s_admin = s_admin  # UI 파일 참조
        self.setWindowTitle("Robot Controller")

        # 제어 방법 설정 (조이스틱)
        #self.control_state = state

        self.robot_mover = RobotMover()  # 로봇 제어 객체 생성

        self.setup_ui()

    def setup_ui(self):
        self.setGeometry(100, 100, 400, 300)
        self.joystick_control()  # 조이스틱 제어 함수 호출

    def joystick_control(self):
        """조이스틱 입력을 받아서 로봇을 비례제어하는 함수"""
        while True:
            events = inputs.get_gamepad()  # 조이스틱 입력을 받아옴
            for event in events:
                if event.ev_type == 'Key':
                    # 축 X (왼쪽/오른쪽) 제어
                    if event.ev_type == 'Absolute':
                        if event.ev_code == 'ABS_X':
                            linear_x = event.ev_value / 32767.0 * 0.5  # X축을 기준으로 linear.x 값 설정

                        # 축 Y (앞뒤) 제어
                        if event.ev_code == 'ABS_Y':
                            angular_z = event.ev_value / 32767.0 * 0.5  # Y축을 기준으로 angular.z 값 설정

                        # 비례적으로 로봇에 명령을 보냄
                        self.robot_mover.send_command(linear_x=linear_x, angular_z=angular_z)

                        self.get_logger().info(f'Joystick Command: linear.x={linear_x}, angular.z={angular_z}')


# 메인 함수
def main():
    rclpy.init()  # ROS 2 초기화

    # PyQt5 애플리케이션 생성
    app = QApplication([])

    # 실제 UI 객체를 여기에 전달해야 합니다.
    s_admin = None  # 실제 s_admin 객체를 여기에 대입해야 합니다.
    
    # 제어 방식 설정 (조이스틱)
    control_state = 'joystick'
    window = JoyStickControl(s_admin)
    window.show()

    # PyQt5 이벤트 루프 실행
    app.exec_()

    # 종료 시 ROS 2 종료
    rclpy.shutdown()


if __name__ == '__main__':
    main()
