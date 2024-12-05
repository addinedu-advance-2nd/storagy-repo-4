import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BatteryListener(Node):
    def __init__(self, s_admin):
        super().__init__('battery_listener')  # 노드 이름 지정
        self.s_admin = s_admin
        #self.s_admin = s_admin  # QLabel 등 UI 객체 참조
        self.subscription = self.create_subscription(
            String,
            "/battery_voltage",
            self.listener_callback,
            10  # 큐 크기
        )
        self.voltage = None

    def listener_callback(self, msg):
        self.voltage = round(float(msg.data), 2)  # 문자열을 float로 변환하고 반올림
        #for i in range(0,100):
        #    self.s_admin.battery_voltage.setText(f"배터리 잔량 : {i} %")
        self.s_admin.battery_voltage.setText(f"배터리 잔량 : {self.voltage} %")
        self.get_value()

    def get_value(self):
        print(self.voltage)
        return self.voltage

    

        '''
        """토픽에서 받은 배터리 상태 메시지를 처리"""
        try:
            voltage = round(float(msg.data), 2)  # 문자열을 float로 변환하고 반올림
            self.get_logger().info(f"Battery voltage: {voltage} V")

        
            # QLabel에 값 출력
            if self.s_admin:
                self.s_admin.battery_voltage.setText(f"배터리 전압: {voltage} V")
        except ValueError:
            self.get_logger().error(f"Invalid data received: {msg.data}")
            
            # QLabel에 오류 메시지 출력
            if self.s_admin:
                self.s_admin.battery_voltage.setText(f"Invalid data: {msg.data}")
            '''

def main(args=None):
    rclpy.init(args=args)
    #s_admin = None  # UI 객체 참조가 없는 경우 기본값 설정
    node = BatteryListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
