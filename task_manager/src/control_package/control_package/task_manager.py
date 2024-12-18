import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from control_msgs.msg import RobotRecevieMoving, RobotRequestMoving, RobotArriveState, AlertToChat
from std_msgs.msg import Bool, String
from array import array
from datetime import datetime
import time
import json
#python -m pip install "pymongo[srv]"==3.6

"""
기능1) 디스코봇에서 간식,사무용품 호출
기능2) 디스코봇으로 프린터 요청하고, 전달 받는 기능

"""
#좌표값
PositionDict = {'init':[0.018, -0.027],
                'print':[0.018, -0.027],
                'corridor1':[0.018, -0.027],
                'corridor2':[2.33, -0.01],
                'corridor3':[3.93, -4.97],
                'A1':[0.018, -0.027,0.28, 0.79],
                'A2':[0.018, -0.027,0.31, 1.64],
                'B1':[2.33, -0.01,2.45, 0.83],
                'B2':[2.33, -0.01,2.4, 1.56],
                'C1':[3.93, -4.97,4.54, 0.7],
                'C2':[3.93, -4.97,4.56, 1.3],
                '신정수':[0.018, -0.027,0.28, 0.79],
                '신정수_B':[0.018, -0.027],
                '김영배':[0.018, -0.027,0.31, 1.64],
                '김영배_B':[0.018, -0.027],
                '안태규':[2.33, -0.01,2.45, 0.83],
                '안태규_B':[2.33, -0.01],
                '황정민':[2.33, -0.01,2.4, 1.56],
                '황정민_B':[2.33, -0.01],
                '김세현':[3.93, -4.97,4.54, 0.7],
                '김세현_B':[3.93, -4.97],
                '이강찬':[3.93, -4.97,4.56, 1.3] ,     
                '이강찬_B':[3.93, -4.97]                             
                }



class TaskManager(Node):
    
    def __init__(self):     
        super().__init__('task_manager')

          
        self.isAvailable = True #스토리지 사용 가능 여부 ( 만약 프린터 시스템에서 사용중일 경우, True 처리 시키기)
        self.isArrived = False #스토리지 이동 완료 
        self.result_msg = None
        self.isPrintComplete = False #프린트완료 여부
        
        self.requset_queue = []

        # 구독 - 스토리지 이동 요청 받기 (채팅봇, 프린터기 -> 매니저)
        self.receive_move_subscriber = self.create_subscription(RobotRecevieMoving, '/moving_receive', self.receive_request_callback, 10)     
        self.receive_move_subscriber  

        # 발행 - 스토리지 이동 요청 하기 ( 매니저 -> 스토리지)
        self.request_move_publisher = self.create_publisher(RobotRequestMoving, '/moving_request', 10)     

        # # 구독 - 스토리지 목적지 도착 완료 받기 (스토리지 -> 매니저)
        # self.receive_arrived_subscriber = self.create_subscription(RobotArriveState, '/arrived_receive', self.arrived_callback, 10)
        # self.receive_arrived_subscriber

        # 발행 - 프린터 요청( 스토리지 프린터기 앞 도착 완료) ( 매니저 -> 프린터시스템)
        self.request_print_publisher = self.create_publisher(RobotRecevieMoving, '/print_request', 10)    

        # # 구독 - 프런터 완료 알림 (프린터기 -> 매니저)
        # self.complete_print_subscriber = self.create_subscription(String, '/print_complete', self.print_complete_callback, 10)
        # self.complete_print_subscriber


    #스토리지 이동 요청이 들어왔을 경우
    def receive_request_callback(self, msg):     
        print_file_path = msg.file_path
        self.isAvailable = False #    
        self.get_logger().info('Request System: "%s", User Name: "%s"' % (msg.request_system, msg.user_name))  
       #self.excute_moving(self, msg)

    
    #def excute_moving(self, msg):     
        # print_file_path = msg.file_path
        # self.isAvailable = False #    
        # #self.get_logger().info('Request System: "%s", User Name: "%s"' % (msg.request_system, msg.user_name))  
        # self.get_logger().info(f"Request Detail: '{msg}'")

        ###########################################################################
         # 스토리지 이동 요청지 - 프린터 일 경우 
        ###########################################################################
        if msg.request_system == "print":   
            print('프린터 시스템에서 호출') 

            # ROS 발행 - 1.스토리지 프린터기로 이동 요청
            send_msg = RobotRequestMoving()             
            send_msg.request_system = msg.request_system  # 메시지 내용              
            send_msg.user_name = msg.user_name # 메시지 내용     
            send_msg.positions = array('f', PositionDict['print']) # 메시지 내용            
            self.request_move_publisher.publish(send_msg)           
            self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))     
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")

            # ROS 구독 - 2.스토리지 도착 완료 
            self.isArrived = False #스토리지 이동 완료 
            self.get_logger().info(f"Publishing isArrived: '{send_msg}'")
            while not self.isArrived:     
                self.get_logger().info(f"while")        
                # rp.spin_once(self)

            self.get_logger().info(f"while 탈출")
            # ROS 발행- 3.프린터시스템에 프린터 요청            
            send_msg = RobotRecevieMoving()
            send_msg.request_system = msg.request_system   # 메시지 내용  
            send_msg.user_name = msg.user_name # 메시지 내용  
            send_msg.file_path = print_file_path  # 메시지 내용  
           
            self.request_print_publisher.publish(send_msg)
            self.get_logger().info('Publishing: "%s"' % send_msg.file_path)         
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")
           
            # ROS 구독 - 4.프린터 시스템 인쇄 완료 
            self.isPrintComplete = False 
            self.result_msg = None
            while not self.isPrintComplete:  
                pass  
                # rp.spin_once(self)
           
            if 'comp' in self.result_msg:
                # ROS 발행 - 5.스토리지 사용자에게 이동 요청
                send_msg = RobotRequestMoving()
                send_msg.request_system = msg.request_system  # 메시지 내용
                send_msg.user_name = msg.user_name  # 메시지 내용              
                send_msg.positions = array('f',PositionDict[msg.user_name])# 메시지 내용
                self.request_move_publisher.publish(send_msg)        
                self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))
                self.get_logger().info(f"Publishing Detail: '{send_msg}'")    
                # ROS 구독 - 6.스토리지 사용자에게  도착 완료 
                self.isArrived = False #스토리지 이동 완료 
                while not self.isArrived:                  
                    # rp.spin_once(self)
                    pass


       

        ###########################################################################
        # 스토리지 이동 요청지 - (프린터 외) 챗봇
        # 스토리지 이동 목적지 - 사용자 자리 OR 홈 OR 복도
        ###########################################################################
        else:
            # ROS 발행 - 스토리지 이동 요청
            send_msg = RobotRequestMoving()
            send_msg.request_system = msg.request_system  # 메시지 내용
            send_msg.user_name = msg.user_name  # 메시지 내용   
            send_msg.positions = array('f',PositionDict[msg.user_name])# 메시지 내용 - 이동    

            self.request_move_publisher.publish(send_msg)        
            self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")    
            
            
            # ROS 구독 - 스토리지 도착 완료 
            self.isArrived = False #스토리지 이동 완료 
            while not self.isArrived:
                # rp.spin_once(self)
                pass


        #이동 완료 후 상태값들 초기화
        self.init_vars()
    

        
    # def arrived_callback(self, msg):
    #     self.get_logger().info('I heard arrived_callback : "%s" ' %( msg.user_name))     
    #     # 구독한 메시지를 출력     
    #     self.isArrived = True #스토리지 이동 완료 
    #     self.result_msg = msg
       
        
    # def print_complete_callback(self, msg):
    #     # 프린터 완료 
    #     self.isPrintComplete = True #스토리지 이동 완료 
    #     self.result_msg = msg
    #     self.get_logger().info('I heard print_complete_callback : "%s" ' %( msg.data))    

    def init_vars(self):              
        self.isAvailable = True #스토리지 사용 가능 여부 ( 만약 프린터 시스템에서 사용중일 경우, True 처리 시키기)
        self.isArrived = False #스토리지 이동 완료 
        self.result_msg = None
        self.isPrintComplete = False #프린트완료 여부

class ArriveSubscriber(Node):
    def __init__(self, taskmanager):
        super().__init__('arrive_subscriber_node')
        # 구독 - 스토리지 목적지 도착 완료 받기 (스토리지 -> 매니저)
        self.receive_arrived_subscriber = self.create_subscription(RobotArriveState, '/arrived_receive', self.arrived_callback, 10)
        self.receive_arrived_subscriber

        self.task_manager = taskmanager
    
    def arrived_callback(self, msg):
        self.get_logger().info('I heard arrived_callback : "%s" ' %( msg.user_name))     
        # 구독한 메시지를 출력     
        self.task_manager.isArrived = True #스토리지 이동 완료 
        self.task_manager.result_msg = msg

class PrinterSubscriber(Node):
    def __init__(self, taskmanager):
        super().__init__('printer_subscriber_node')
        # 구독 - 스토리지 목적지 도착 완료 받기 (스토리지 -> 매니저)
        self.complete_print_subscriber = self.create_subscription(String, '/print_complete', self.print_complete_callback, 10)
        self.complete_print_subscriber

        self.task_manager = taskmanager
    
    def print_complete_callback(self, msg):
        # 프린터 완료 
        self.task_manager.isPrintComplete = True #스토리지 이동 완료 
        self.task_manager.result_msg = msg
        self.get_logger().info('I heard print_complete_callback : "%s" ' %( msg.data))    
       
     
        
def main(args=None):
    rp.init(args=args)
    task_manager = TaskManager()
    arrive_subscriber = ArriveSubscriber(task_manager)
    executor = MultiThreadedExecutor()
    executor.add_node(task_manager)
    executor.add_node(arrive_subscriber)
  
    try:
        executor.spin()

    finally:
        executor.shutdown()
        task_manager.destroy_node()
        arrive_subscriber.destroy_node()
       
        rp.shutdown()     

if __name__ == '__main__':
    main()