import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QMainWindow, QApplication, QPushButton
from PyQt5.QtCore import Qt, QThread
import sys
from PyQt5.QtCore import QTimer

# ROS 2 노드 (CmdVelPub)
class CmdVelPub(Node):
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

'''
# ROS 2 스핀을 실행할 스레드
class RclpyThread(QThread):
    def run(self):
        rclpy.spin(self.node)  # ROS 2 spin을 별도의 스레드에서 실행

    def __init__(self, node):
        super().__init__()
        self.node = node
'''

        # 메인 함수
def main():
    rp.init()  # ROS 2 초기화

    # PyQt5 애플리케이션 생성
    app = QApplication([])

    # UI 객체를 전달해야 합니다. 실제 UI 객체를 사용하세요.
    #cmd_vel_value = None  # 실제 s_admin 객체를 여기에 전달

    # 제어 방식 설정 (keyboard)
    window = CmdVelPub()
    #window.show()

    window.send_command()

    # PyQt5 이벤트 루프 실행
    app.exec_()

    # 종료 시 ROS 2 종료
    rp.shutdown()


if __name__ == '__main__':
    main()
