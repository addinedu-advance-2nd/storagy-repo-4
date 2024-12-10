import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QThread, pyqtSignal

# 로깅 설정
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# RobotMover 클래스 (ROS 2 노드)
class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        logger.info('Robot Mover Node Initialized')

    def send_command(self, linear_x=0.0, angular_z=0.0):
        move_cmd = Twist()
        move_cmd.linear.x = linear_x
        move_cmd.angular.z = angular_z
        self.publisher_.publish(move_cmd)
        logger.info(f'Publishing: linear.x={linear_x}, angular.z={angular_z}')

# JoyStickControl 클래스 (PyQt5 GUI, 조이스틱 제어)
class JoyStickControl(QMainWindow):
    def __init__(self, s_admin):
        super().__init__()
        self.s_admin = s_admin
        self.setWindowTitle("Robot Controller")
        self.robot_mover = RobotMover()  # ROS 2 노드 객체 생성
        self.setup_ui()

    def setup_ui(self):
        self.setGeometry(100, 100, 400, 300)
        self.joystick_control()

    def joystick_control(self):
        """조이스틱 입력을 받아서 로봇을 비례제어하는 함수"""
        pygame.init()  # Pygame 초기화
        joystick = pygame.joystick.Joystick(0)  # 첫 번째 조이스틱 연결
        joystick.init()  # 조이스틱 초기화

        std_speed = 0.2
        
        while True:
            events = pygame.event.get()  # Pygame 이벤트를 받아옴
            for event in events:
                if event.type == pygame.JOYAXISMOTION:
                    linear_x = joystick.get_axis(1)  # Y축 (앞뒤) 제어
                    angular_z = joystick.get_axis(0)  # X축 (왼쪽/오른쪽) 제어

                    # 속도 방향 보정
                    if linear_x < -0.8:
                        linear_x = std_speed
                    elif linear_x > 0.8:
                        linear_x = std_speed*(-1)
                    else :
                        linear_x = 0.0

                    if angular_z > 0.8:
                        angular_z = std_speed
                    elif angular_z < -0.8:
                        angular_z = std_speed * (-1)
                    else:
                        angular_z = 0.0




                    self.robot_mover.send_command(linear_x=linear_x, angular_z=angular_z)

                    logger.info(f'Joystick Command: linear.x={linear_x}, angular.z={angular_z}')
        
        pygame.quit()  # Pygame 종료

# 메인 함수
def main():
    rclpy.init()  # ROS 2 초기화

    app = QApplication([])  # PyQt5 애플리케이션 생성
    s_admin = None  # 실제 s_admin 객체를 여기에 대입
    window = JoyStickControl(s_admin)  # JoyStickControl 객체 생성
    window.show()  # GUI 윈도우 표시

    app.exec_()  # PyQt5 이벤트 루프 실행

    rclpy.shutdown()  # 종료 시 ROS 2 종료

if __name__ == '__main__':
    main()
