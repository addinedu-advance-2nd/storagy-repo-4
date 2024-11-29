import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Ros2MonitorNode(Node):
    def __init__(self, s_admin):
        super().__init__('ros2_monitor_node')
        self.topic_list = []  # 현재 실행 중인 토픽 목록
        self.s_admin = s_admin  # s_admin 인스턴스를 받아옵니다
        self.update_topics()  # 토픽 목록 업데이트

        # 배터리 상태를 구독
        self.battery_listener()

        # 속도 명령을 구독
        self.cmd_vel_listener()
        self.cmd_vel_nav_listener()

    def update_topics(self):
        # 현재 활성화된 토픽 목록 갱신
        self.topic_list = self.get_topic_names_and_types()
        self.get_logger().info(f"Active topics: {self.topic_list}")

    def battery_listener(self):
        # 배터리 상태 토픽을 구독
        self.subscription = self.create_subscription(
            String,           # 구독할 메시지 타입
            '/battery_voltage',  # 구독할 토픽 이름
            self.listener_callback,  # 콜백 함수
            10  # 큐 크기
        )

    def listener_callback(self, msg):
        # 배터리 상태 메시지를 처리
        try:
            voltage = float(msg.data)  # 문자열을 float로 변환
            voltage_rounded = round(voltage, 2)  # 소수점 두 번째 자리까지 반올림
            self.get_logger().info(f"Battery voltage: {voltage_rounded} V")
            
            # s_admin에서 QLabel인 volt에 값 출력
            self.s_admin.battery_voltage.setText(f"배터리 전압: {voltage_rounded} V")
        except ValueError:
            self.get_logger().error(f"Received invalid data: {msg.data}")
            self.s_admin.battery_voltage.setText(f"Received invalid data: {msg.data}")

    def cmd_vel_listener(self):
        # /cmd_vel 토픽을 구독
        self.subscription = self.create_subscription(
            Twist,           # 구독할 메시지 타입
            '/cmd_vel',      # 구독할 토픽 이름
            self.cmd_vel_callback,  # 콜백 함수
            10  # 큐 크기
        )

    def cmd_vel_callback(self, msg):
        # Linear 및 Angular 속도 값을 로그로 출력
        self.get_logger().info(f"Linear Velocity - x: {msg.linear.x}, y: {msg.linear.y}, z: {msg.linear.z}")
        self.get_logger().info(f"Angular Velocity - x: {msg.angular.x}, y: {msg.angular.y}, z: {msg.angular.z}")
        
        # s_admin에서 QLabel인 cmd_vel에 값 출력
        self.s_admin.cmd_vel.setText(f"Linear Velocity - x: {msg.linear.x}, y: {msg.linear.y}, z: {msg.linear.z}")
        self.s_admin.cmd_vel.setText(f"Angular Velocity - x: {msg.angular.x}, y: {msg.angular.y}, z: {msg.angular.z}")

    def cmd_vel_nav_listener(self):
        # /cmd_vel_nav 토픽을 구독
        self.subscription = self.create_subscription(
            Twist,           # 구독할 메시지 타입
            '/cmd_vel_nav',  # 구독할 토픽 이름
            self.cmd_vel_nav_callback,  # 콜백 함수
            10  # 큐 크기
        )

    def cmd_vel_nav_callback(self, msg):
        # Linear 및 Angular 속도 값을 로그로 출력
        self.get_logger().info(f"Nav Linear Velocity - x: {msg.linear.x}, y: {msg.linear.y}, z: {msg.linear.z}")
        self.get_logger().info(f"Nav Angular Velocity - x: {msg.angular.x}, y: {msg.angular.y}, z: {msg.angular.z}")

        # s_admin에서 QLabel인 cmd_vel_nav에 값 출력
        self.s_admin.cmd_vel_nav.setText(f"Nav Linear Velocity - x: {msg.linear.x}, y: {msg.linear.y}, z: {msg.linear.z}")
        self.s_admin.cmd_vel_nav.setText(f"Nav Angular Velocity - x: {msg.angular.x}, y: {msg.angular.y}, z: {msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    
    # s_admin은 PyQt에서 GUI 관리하는 객체입니다. 이를 전달하여 노드를 초기화합니다.
    s_admin = None  # 실제로 s_admin을 PyQt 클래스에서 전달해야 합니다.
    
    # ROS2 모니터 노드 생성
    ros2_monitor_node = Ros2MonitorNode(s_admin)
    
    # 노드 실행
    rclpy.spin(ros2_monitor_node)
    
    # 종료
    ros2_monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
