import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TfListener(Node):
    def __init__(self, s_admin):
        super().__init__('tf_listener')  # 노드 이름 지정
        self.s_admin = s_admin
        #self.s_admin = s_admin  # QLabel 등 UI 객체 참조
        self.subscription = self.create_subscription(
            TFMessage,
            "/tf",
            self.listener_callback,
            1  # 큐 크기
        )
        self.voltage = None

    def listener_callback(self, msg):
        #print(msg)
        for transform in msg.transforms:
            # 기준 좌표 프레임
            frame_id = transform.header.frame_id
            # 대상 좌표 프레임
            child_frame_id = transform.child_frame_id
            # 위치
            translation = transform.transform.translation
            # 회전
            rotation = transform.transform.rotation
            
            #self.get_logger().info(f"Transform from {frame_id} to {child_frame_id}:")
            #self.get_logger().info(f"  Translation: x={translation.x}, y={translation.y}, z={translation.z}")
            #self.get_logger().info(f"  Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}")

        
        # 소수점 2자리로 정리
        formatted_position = {
            "x": round(translation.x, 1),
            "y": round(translation.y, 1),
            "z": round(translation.z, 1)
        }

        formatted_orientation = {
            "x": round(rotation.x, 1),
            "y": round(rotation.y, 1),
            "z": round(rotation.z, 1),
            "w": round(rotation.w, 1)
        }

        # 결과 출력
        formatted_output = (
        "tf\n"
        f"position: x: {formatted_position['x']}, y: {formatted_position['y']}, z: {formatted_position['z']}\n"
        f"orientation: x: {formatted_orientation['x']}, y: {formatted_orientation['y']}, z: {formatted_orientation['z']}, w: {formatted_orientation['w']}"
        )

        # QLabel에 텍스트 설정
        #self.s_admin.odom.setText(formatted_output)
        self.s_admin.tf_pos_x.setText(f"x:{formatted_position['x']}")
        self.s_admin.tf_pos_y.setText(f"y:{formatted_position['y']}")
        self.s_admin.tf_pos_z.setText(f"z:{formatted_position['z']}")
        self.s_admin.tf_ori_x.setText(f"x:{formatted_orientation['x']}")
        self.s_admin.tf_ori_y.setText(f"x:{formatted_orientation['y']}")
        self.s_admin.tf_ori_z.setText(f"z:{formatted_orientation['z']}")
        self.s_admin.tf_ori_w.setText(f"w:{formatted_orientation['w']}")
        
      
def main(args=None):
    rclpy.init(args=args)
    s_admin = None  # UI 객체 참조가 없는 경우 기본값 설정
    node = TfListener(s_admin)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
