import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from control_msgs.msg import RobotRecevieMoving

class PrintPublisher(Node):
    def __init__(self):
        super().__init__('print_publisher')  # 노드 이름
        self.publisher_ = self.create_publisher(RobotRecevieMoving, '/moving_receive', 10)  # 주제 이름    
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1초 간격으로 호출

    def timer_callback(self):
        msg = RobotRecevieMoving()
        msg.request_system = 'print'  # 메시지 내용
        msg.user_name = '안태규'  # 메시지 내용
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.request_system)

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    publisher = PrintPublisher()

    rclpy.spin(publisher)  # 노드 실행

    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()