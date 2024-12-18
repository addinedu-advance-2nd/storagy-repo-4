import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber_node')

        # QoS 설정 (Reliability를 RELIABLE로 설정, Depth는 1)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 'map' 토픽 구독
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            qos_profile
        )
        self.get_logger().info('Map subscriber node initialized. Waiting for map data...')
        
        # 히트맵 설정
        self.fig, self.ax = plt.subplots()
        plt.ion()  # Interactive mode ON for live updates
        self.heatmap = None

    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info('Received map data.')
        print(msg)
        # OccupancyGrid 데이터를 NumPy 배열로 변환
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))  # 2D 배열로 변환
        
        # -1 값을 255 (흰색)으로, 0 값을 그대로 두고 나머지 값은 조정
        data = np.where(data == -1, 255, data)  # Unknown(-1)을 흰색(255)으로 설정
        data = np.where(data == 0, 0, data)    # Free(0)은 검은색
        data = np.clip(data, 0, 100)           # 나머지 값 (0~100) 범위로 유지
        print(data)

        # 히트맵 표시
        if self.heatmap is None:
            self.heatmap = self.ax.imshow(data, cmap='gray', origin='upper', vmin=0, vmax=255)
            self.fig.colorbar(self.heatmap, ax=self.ax)
            self.ax.set_title("Real-Time Map Heatmap")
        else:
            self.heatmap.set_data(data)

        plt.draw()
        plt.pause(0.1)  # 업데이트 지연 시간


def main(args=None):
    rclpy.init(args=args)

    # 노드 실행
    map_subscriber = MapSubscriber()
    try:
        rclpy.spin(map_subscriber)
    except KeyboardInterrupt:
        map_subscriber.get_logger().info('Shutting down Map Subscriber Node...')
    finally:
        map_subscriber.destroy_node()
        rclpy.shutdown()
        plt.close()


if __name__ == '__main__':
    main()
