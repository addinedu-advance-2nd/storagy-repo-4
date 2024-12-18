import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage  # This is the message type for the /tf topic

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,  # The message type for the /tf topic
            '/tf',  # The topic name
            self.listener_callback,
            10  # Queue size
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        for transform in msg.transforms:
            self.get_logger().info(f"Received transformation: {transform}")

def main(args=None):
    rclpy.init(args=args)

    tf_subscriber = TFSubscriber()

    rclpy.spin(tf_subscriber)

    tf_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
