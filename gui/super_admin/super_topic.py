import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TopicSubscriber(Node):

    def __init__(self):
        super().__init__('topic_subscriber')  # 노드 이름
        self.subscription = self.create_subscription(
            String,              # 구독할 메시지 타입
            '/rosout',        # 구독할 토픽 이름
            self.listener_callback,  # 콜백 함수
            10                    # 큐 크기
        )

    def listener_callback(self, msg):
        self.get_logger().info('Received message: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    subscriber = TopicSubscriber()

    # 노드 실행
    rclpy.spin(subscriber)

    # 종료 시 클린업
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
