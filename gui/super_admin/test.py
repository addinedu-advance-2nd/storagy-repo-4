import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BatterySubscriber(Node):

    def __init__(self):
        super().__init__('battery_subscriber')
        # 배터리 상태를 구독합니다.
        self.subscription = self.create_subscription(
            String,
            '/battery_voltage',  # 구독할 토픽 이름
            self.battery_callback,
            10
        )

    def battery_callback(self, msg):
        # 배터리 상태 정보를 터미널에 출력합니다.
        print(msg)
        '''
        voltage = msg.voltage
        current = msg.current
        charge = msg.charge
        percentage = msg.percentage

        self.get_logger().info(f"Voltage: {voltage} V")
        self.get_logger().info(f"Current: {current} A")
        self.get_logger().info(f"Charge: {charge} Ah")
        self.get_logger().info(f"Percentage: {percentage * 100}%")
        '''

def main(args=None):
    rclpy.init(args=args)

    # 노드 생성
    battery_subscriber = BatterySubscriber()

    # 노드 실행
    rclpy.spin(battery_subscriber)

    # 종료
    battery_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
