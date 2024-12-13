import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from control_msgs.msg import RobotRecevieMoving, RobotRequestMoving, RobotArriveState, AlertToChat
from std_msgs.msg import Bool, String
from array import array
from datetime import datetime
import time
import json

"""
기능1) 디스코봇에서 간식,사무용품 호출
기능2) 디스코봇으로 프린터 요청하고, 전달 받는 기능

"""
#좌표값
PositionDict = {'init':[0, 0],
                'print':[0, 1],
                'corridor1':[0.22, 0.59],
                'corridor2':[2.3, 0.47],
                'corridor3':[3.5, 0.78],
                'A1':[0.22, 0.59,0.17, 1.71],
                'A2':[0.22, 0.59,0.16, 2.4],
                'B1':[2.3, 0.47,2.3, 1.67],
                'B2':[2.3, 0.47,2.3, 2.45],
                'C1':[3.5, 0.78,4.05, 1.05],
                'C2':[3.5, 0.78,1.05, 2]                
                }
UserKeyDict={
            "신정수":"A1",
            "김영배":"A2",
            "안태규":"B1",
            "황정민":"B2",
            "김세현":"C1",
            "이강찬":"C2",
            "A1":"A1"

}


class TaskManager(Node):
    
    def __init__(self):     
        super().__init__('task_manager')

          
        self.isAvailable = True #스토리지 사용 가능 여부 ( 만약 프린터 시스템에서 사용중일 경우, True 처리 시키기)
        self.isArrived = False #스토리지 이동 완료 
        self.result_msg = None
        self.isPrintComplete = False #프린트완료 여부
     

        # 구독 - 스토리지 이동 요청 받기 (채팅봇, 프린터기 -> 매니저)
        self.receive_move_subscriber = self.create_subscription(RobotRecevieMoving, '/moving_receive', self.receive_request_callback, 10)     
        self.receive_move_subscriber  

        # 발행 - 스토리지 이동 요청 하기 ( 매니저 -> 스토리지)
        self.request_move_publisher = self.create_publisher(RobotRequestMoving, '/moving_request', 10)     

        # 구독 - 스토리지 목적지 도착 완료 받기 (스토리지 -> 매니저)
        self.receive_arrived_subscriber = self.create_subscription(RobotArriveState, '/arrived_receive', self.arrived_callback, 10)
        self.receive_arrived_subscriber

        # 발행 - 프린터 요청( 스토리지 프린터기 앞 도착 완료) ( 매니저 -> 프린터시스템)
        self.request_print_publisher = self.create_publisher(RobotRecevieMoving, '/print_request', 10)    

        # 구독 - 프런터 완료 알림 (프린터기 -> 매니저)
        self.complete_print_subscriber = self.create_subscription(String, '/print_complete', self.print_complete_callback, 10)
        self.complete_print_subscriber


    #스토리지 이동 요청이 들어왔을 경우
    def receive_request_callback(self, msg):     
        print_file_path = msg.file_path
        self.isAvailable = False #    
        self.get_logger().info('Request System: "%s", User Name: "%s"' % (msg.request_system, msg.user_name))  
      
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
        
 
            # ROS 구독 - 2.스토리지 도착 완료 
            self.isArrived = False #스토리지 이동 완료 
            while not self.isArrived:              
                rp.spin_once(self)
            
            # ROS 발행- 3.프린터시스템에 프린터 요청            
            send_msg = RobotRecevieMoving()
            send_msg.request_system = msg.request_system   # 메시지 내용  
            send_msg.user_name = msg.user_name # 메시지 내용  
            send_msg.file_path = print_file_path  # 메시지 내용  
           
            self.request_print_publisher.publish(send_msg)
            self.get_logger().info('Publishing: "%s"' % send_msg.file_path)         

           
            # ROS 구독 - 4.프린터 시스템 인쇄 완료 
            self.isPrintComplete = False 
            self.result_msg = None
            while not self.isPrintComplete:    
                rp.spin_once(self)
           
            if 'comp' in self.result_msg:
                # ROS 발행 - 5.스토리지 사용자에게 이동 요청
                send_msg = RobotRequestMoving()
                send_msg.request_system = msg.request_system  # 메시지 내용
                send_msg.user_name = msg.user_name  # 메시지 내용
                send_msg.positions = array('f',PositionDict[UserKeyDict[msg.user_name]])# 메시지 내용
                self.request_move_publisher.publish(send_msg)        
                self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))

                # ROS 구독 - 6.스토리지 사용자에게  도착 완료 
                self.isArrived = False #스토리지 이동 완료 
                while not self.isArrived:                  
                    rp.spin_once(self)

        ###########################################################################
        # 스토리지 이동 요청지 - 프린터 외 챗봇
        ###########################################################################
        else:
            # ROS 발행 - 스토리지 이동 요청
            send_msg = RobotRequestMoving()
            send_msg.request_system = msg.request_system  # 메시지 내용
            send_msg.user_name = msg.user_name  # 메시지 내용
            send_msg.positions = array('f',PositionDict[UserKeyDict[msg.user_name]])# 메시지 내용
            self.request_move_publisher.publish(send_msg)        
            self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))

            # ROS 구독 - 스토리지 도착 완료 
            self.isArrived = False #스토리지 이동 완료 
            while not self.isArrived:
                rp.spin_once(self)


        #이동 완료 후 상태값들 초기화
        self.init_vars()
 
        
    def arrived_callback(self, msg):
        # 구독한 메시지를 출력     
        self.isArrived = True #스토리지 이동 완료 
        self.result_msg = msg
        self.get_logger().info('I heard arrived_callback : "%s" ' %( msg.user_name))    
    
        
    def print_complete_callback(self, msg):
        # 프린터 완료 
        self.isPrintComplete = True #스토리지 이동 완료 
        self.result_msg = msg
        self.get_logger().info('I heard print_complete_callback : "%s" ' %( msg.data))    

    def init_vars(self):              
        self.isAvailable = True #스토리지 사용 가능 여부 ( 만약 프린터 시스템에서 사용중일 경우, True 처리 시키기)
        self.isArrived = False #스토리지 이동 완료 
        self.result_msg = None
        self.isPrintComplete = False #프린트완료 여부
     
        
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