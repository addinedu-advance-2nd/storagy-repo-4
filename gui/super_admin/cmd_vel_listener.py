import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CmdVelListener(Node):
    def __init__(self, s_admin):
        super().__init__('cmd_vel_listener')  # 노드 이름 지정
        self.s_admin = s_admin
        #self.s_admin = s_admin  # QLabel 등 UI 객체 참조
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.listener_callback,
            10  # 큐 크기
        )
        self.voltage = None

    def listener_callback(self, msg):
        #print(msg)

        # linear 및 angular 값 추출 및 소수점 2자리로 포맷팅
        linear_x = round(msg.linear.x, 2)
        linear_y = round(msg.linear.y, 2)
        linear_z = round(msg.linear.z, 2)
        angular_x = round(msg.angular.x, 2)
        angular_y = round(msg.angular.y, 2)
        angular_z = round(msg.angular.z, 2)
        
        # 텍스트 형식 설정
        formatted_text = ("Command Velocity(cmd_vel)\n"
            f"Linear: x={linear_x}, y={linear_y}, z={linear_z}\n"
                          f"Angular: x={angular_x}, y={angular_y}, z={angular_z}")
        
        # QLabel에 업데이트
        #self.s_admin.cmd_vel.setText(formatted_text)
        self.s_admin.cmd_vel_x.setText(str(linear_x))
        self.s_admin.cmd_vel_z.setText(str(angular_z))
        #print(linear_x)
        
def main(args=None):
    rclpy.init(args=args)
    s_admin = None  # UI 객체 참조가 없는 경우 기본값 설정
    node = CmdVelListener(s_admin)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
