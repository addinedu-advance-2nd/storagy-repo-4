import rclpy
from rclpy.executors import MultiThreadedExecutor
from print_node import PrintNode
from flask_server import create_app  # Flask 앱을 가져옵니다.

def main(args=None):
    rclpy.init(args=args)

    # ROS 노드 초기화
    print_node = PrintNode()

    # 다중 스레드 실행기를 사용하여 ROS 노드와 Flask 서버를 함께 실행
    executor = MultiThreadedExecutor()
    executor.add_node(print_node)


    # Flask 앱 생성 (PrintNode를 전달)
    app = create_app(print_node)


    try:
        # Flask 서버를 별도의 스레드에서 실행
        from threading import Thread
        flask_thread = Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000})
        flask_thread.start()

        # ROS 노드 실행
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        print_node.destroy_node()
        rclpy.shutdown()
        # Flask 서버 종료를 위한 스레드 종료
        flask_thread.stop()
        flask_thread.join()  # 스레드가 종료될 때까지 대기

if __name__ == "__main__":
    main()
