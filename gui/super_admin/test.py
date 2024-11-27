import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

class BatteryListener(Node):
    def __init__(self):
        super().__init__('battery_listener')
        # '/battery_voltage' 토픽을 구독
        self.subscription = self.create_subscription(
            String,           # 구독할 메시지 타입
            '/battery_voltage',       # 구독할 토픽 이름
            self.listener_callback, # 콜백 함수
            10                       # 큐 크기
        )

    def listener_callback(self, msg):
        # 토픽에서 받은 배터리 상태 메시지를 처리
        print(msg)

        # String 메시지에서 데이터를 추출하고 소수점 두 번째 자리까지 반올림
        try:
            voltage = float(msg.data)  # 문자열을 float로 변환
            voltage_rounded = round(voltage, 2)  # 소수점 두 번째 자리까지 반올림
            self.get_logger().info(f"Battery voltage: {voltage_rounded} V")
        except ValueError:
            self.get_logger().error(f"Received invalid data: {msg.data}")
        #self.get_logger().info(f"Battery voltage: {msg.voltage} V")

def main(args=None):
    rclpy.init(args=args)
    
    # 배터리 상태 구독 노드 생성 및 실행
    battery_listener = BatteryListener()
    
    # ROS 2 이벤트 루프 실행
    rclpy.spin(battery_listener)

    # 종료 시 ROS 2 정리
    battery_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
