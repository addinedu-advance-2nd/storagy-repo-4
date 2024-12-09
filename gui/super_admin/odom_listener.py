import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomListener(Node):
    def __init__(self, s_admin):
        super().__init__('odom_listener')  # 노드 이름 지정
        self.s_admin = s_admin
        #self.s_admin = s_admin  # QLabel 등 UI 객체 참조
        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.listener_callback,
            10  # 큐 크기
        )
        self.voltage = None

    def listener_callback(self, msg):

        odom_msg = msg

        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation

        # 소수점 2자리로 정리
        formatted_position = {
            "x": round(position.x, 2),
            "y": round(position.y, 2),
            "z": round(position.z, 2)
        }

        formatted_orientation = {
            "x": round(orientation.x, 2),
            "y": round(orientation.y, 2),
            "z": round(orientation.z, 2),
            "w": round(orientation.w, 2)
        }

        # 결과 출력
        formatted_output = (
        "Odom(odom)\n"
        f"position: x: {formatted_position['x']}, y: {formatted_position['y']}, z: {formatted_position['z']}\n"
        f"orientation: x: {formatted_orientation['x']}, y: {formatted_orientation['y']}, z: {formatted_orientation['z']}, w: {formatted_orientation['w']}"
        )

        # QLabel에 텍스트 설정
        self.s_admin.odom.setText(formatted_output)

      
def main(args=None):
    rclpy.init(args=args)
    s_admin = None  # UI 객체 참조가 없는 경우 기본값 설정
    node = OdomListener(s_admin)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
