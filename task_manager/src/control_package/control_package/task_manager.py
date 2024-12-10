import rclpy as rp
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from control_msgs.msg import RobotRecevieMoving, RobotRequestMoving
from std_msgs.msg import Bool, String

from datetime import datetime
import time
import json


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        
        # 구독 - 샘플 테스트용     
        #self.subscription = self.create_subscription(String,'simple_topic',  self.listener_callback, 10)  # 큐 크기       

        # 구독 - 스토리지 이동 요청 받기 (채팅봇, 프린터기 -> 매니저)
        self.receive_move_subscriber = self.create_subscription(RobotRecevieMoving, '/moving_receive', self.receive_callback, 10)

        # 발행 - 스토리지 이동 요청 하기 ( 매니저 -> 스토리지)
        self.request_move_publisher = self.create_publisher(RobotRequestMoving, '/moving_request', 10)     
        
        self.isAvailable = True #스토리지 이동 가능 여부 ( 만약 프린터 시스템에서 사용중일 경우, False 처리 시키기)

    def receive_callback(self, msg):
        system_name = msg.request_system
        user_name = msg.user_name    

        # 구독한 메시지를 출력
        self.get_logger().info('I heard: "%s"' % system_name)

        print('user_name : '+user_name)           
        if system_name == "print":
            print('프린터 시스템에서 호출')           
               
        else:
            print('챗봇 시스템에서 호출')             

    def listener_callback(self, msg):
        # 구독한 메시지를 출력
        self.get_logger().info('I heard: "%s"' % msg.data)    
        
def main(args=None):
    rp.init(args=args)
    task_manager = TaskManager()
    executor = MultiThreadedExecutor()
    executor.add_node(task_manager)
  
    try:
        executor.spin()

    finally:
        executor.shutdown()
        task_manager.destroy_node()
       
        rp.shutdown()     

if __name__ == '__main__':
    main()