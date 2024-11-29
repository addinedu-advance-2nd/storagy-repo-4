import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TopicListener(Node):
    def __init__(self, topic_name, s_admin=None):
        super().__init__()
        self.topic_name = topic_name  # 구독할 토픽 이름
        self.s_admin = s_admin  # QLabel 등 UI 객체 참조
        self.subscription = self.create_subscription(
            String,
            self.topic_name,
            self.listener_callback,
            10  # 큐 크기
        )
        self.subscription  # 방출되지 않도록 유지

    def listener_callback(self, msg):
        """토픽에서 받은 배터리 상태 메시지를 처리"""
        try:
            voltage = float(msg.data)  # 문자열을 float로 변환
            voltage_rounded = round(voltage, 2)  # 소수점 두 번째 자리까지 반올림
            self.get_logger().info(f"Battery voltage: {voltage_rounded} V")
            
            # QLabel에 값 출력 (UI 연동이 있는 경우)
            if self.s_admin:
                self.s_admin.battery_voltage.setText(f"배터리 전압: {voltage_rounded} V")
        
        except ValueError:
            self.get_logger().error(f"Received invalid data: {msg.data}")
            
            # QLabel에 오류 메시지 출력
            if self.s_admin:
                self.s_admin.battery_voltage.setText(f"Invalid data: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    topic_name = '/battery_voltage'
    node = BatteryListener(topic_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
