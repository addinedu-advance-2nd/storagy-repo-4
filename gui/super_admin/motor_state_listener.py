import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String

class MotorStateListener(Node):
    def __init__(self, s_admin):
        super().__init__('motor_state_listener')  # 노드 이름 지정
        self.s_admin = s_admin
        #self.s_admin = s_admin  # QLabel 등 UI 객체 참조
        self.subscription = self.create_subscription(
            String,
            "/motor_state",
            self.listener_callback,
            10  # 큐 크기
        )
        self.voltage = None

    def listener_callback(self, msg):

       # JSON 데이터 파싱
        motor_data = json.loads(msg.data)

        # 왼쪽 모터 정보
        motor_left_current = motor_data["motor_left"]["current"]
        motor_left_torque = motor_data["motor_left"]["torque"]

        # 오른쪽 모터 정보
        motor_right_current = motor_data["motor_right"]["current"]
        motor_right_torque = motor_data["motor_right"]["torque"]

        # QLabel에 표시할 형식 설정
        formatted_text = (
            f"Motor State(motor_state):\n"
            f"Motor Left: "
            f"  전류: {motor_left_current} A, "
            f"  토크: {motor_left_torque} Nm\n"
            f"Motor Right: "
            f"  전류: {motor_right_current} A, "
            f"  토크: {motor_right_torque} Nm"
        )

        # QLabel에 텍스트 설정
        #self.s_admin.motor_state.setText(formatted_text)
        self.s_admin.motor_r_A.setText(f"{motor_right_current} A")
        self.s_admin.motor_r_Nm.setText(f"{motor_right_torque} Nm")
        self.s_admin.motor_l_A.setText(f"{motor_left_current} A")
        self.s_admin.motor_l_Nm.setText(f"{motor_left_torque} Nm")


def main(args=None):
    rclpy.init(args=args)
    s_admin = None  # UI 객체 참조가 없는 경우 기본값 설정
    node = MotorStateListener(s_admin)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
