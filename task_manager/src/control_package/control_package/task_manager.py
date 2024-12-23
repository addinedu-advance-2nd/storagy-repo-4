import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from control_msgs.msg import RobotRecevieMoving, RobotRequestMoving, RobotArriveState, AlertToChat
from std_msgs.msg import Bool, String
from array import array
from datetime import datetime
import time
import json
from pymongo.mongo_client import MongoClient
import asyncio
from collections import deque

#python -m pip install "pymongo[srv]"==3.6
waiting_time = 10


"""
기능1) 디스코봇에서 간식,사무용품 호출
기능2) 디스코봇으로 프린터 요청하고, 전달 받는 기능

"""
#좌표값
# PositionDict = {'init':[0.018, -0.027],
#                 'print':[0.018, -0.027],
#                 'corridor1':[0.018, -0.027],
#                 'corridor2':[2.33, -0.01],
#                 'corridor3':[3.93, -4.97],
#                 'A1':[0.018, -0.027,0.28, 0.79],
#                 'A2':[0.018, -0.027,0.31, 1.64],
#                 'B1':[2.33, -0.01,2.45, 0.83],
#                 'B2':[2.33, -0.01,2.4, 1.56],
#                 'C1':[3.93, -4.97,4.54, 0.7],
#                 'C2':[3.93, -4.97,4.56, 1.3],
#                 '신정수':[0.018, -0.027,0.28, 0.79],
#                 '신정수_B':[0.018, -0.027],
#                 '김영배':[0.018, -0.027,0.31, 1.64],
#                 '김영배_B':[0.018, -0.027],
#                 '안태규':[2.33, -0.01,2.45, 0.83],
#                 '안태규_B':[2.33, -0.01],
#                 '황정민':[2.33, -0.01,2.4, 1.56],
#                 '황정민_B':[2.33, -0.01],
#                 '김세현':[3.93, -4.97,4.54, 0.7],
#                 '김세현_B':[3.93, -4.97],
#                 '이강찬':[3.93, -4.97,4.56, 1.3] ,     
#                 '이강찬_B':[3.93, -4.97]                             
#                 }


#PositionDict = {'init':[0.018, -0.027]}
PositionDict = {'init':[0, -0]}

class TaskManager(Node):
    
    def __init__(self):     
        super().__init__('task_manager')
        self.task_queue = deque()  # 사용자와 메시지를 저장할 deque

          
        self.isAvailable = True #스토리지 사용 가능 여부 ( 만약 프린터 시스템에서 사용중일 경우, True 처리 시키기)
        self.isArrived = False #스토리지 이동 완료 
        self.result_msg = None
        self.isPrintComplete = False #프린트완료 여부
        
        self.last_user_name = ""
        self.last_user_time = None

        """
        MongoDB Atlas 클라우드 데이터 베이스 연동        
        """
        # MongoDB 초기화
        self.mongo_uri = "mongodb+srv://robopalz:1234@cluster0.4rv3n.mongodb.net/?retryWrites=true&w=majority&appName=Cluster0"
        self.client = MongoClient(self.mongo_uri)
        self.db = self.client["ROBOPALZ"]  # 데이터베이스 이름
        self.collection = self.db["USER_POSITION"]  # 컬렉션 이름

        
        # 구독 - 스토리지 이동 요청 받기 (채팅봇, 프린터기 -> 매니저)
        self.receive_move_subscriber = self.create_subscription(RobotRecevieMoving, '/moving_receive', self.receive_request_callback, 10)     
        self.receive_move_subscriber  

        # 발행 - 스토리지 이동 요청 하기 ( 매니저 -> 스토리지)
        self.request_move_publisher = self.create_publisher(RobotRequestMoving, '/moving_request', 10)     


        # 발행 - 프린터 요청( 스토리지 프린터기 앞 도착 완료) ( 매니저 -> 프린터시스템)
        self.request_print_publisher = self.create_publisher(RobotRecevieMoving, '/print_request', 10)    

    def __del__(self):
        self.client.close()
        self.get_logger().info("MongoDB connection closed.") 

    def get_position_array(self, key):      

        position = self.get_position_from_db(key) #클라우드 DB에서 가져오기

        if position is None:
            position = array('f', PositionDict[key])
            if key in PositionDict:
                # PositionDict에 key가 있을 경우
                position = array('f', PositionDict[key])
            else:
                # PositionDict에 key가 없을 경우 'init' 키 사용
                position = array('f', PositionDict['init'])
       
        return position

    def get_position_from_db(self, key):
        """MongoDB에서 데이터 조회"""
        document = self.collection.find_one({"key": key})
        if document and "value" in document:
            self.get_logger().info(f"Retrieved from DB: {document}")
            return document["value"]
        else:
            self.get_logger().info(f"No data found for key: {key}")
            return None
        
    def add_task(self, user, message):
        # 사용자와 메시지를 deque에 추가
        #self.task_queue.append((user, message))
        self.task_queue.append({ user: message})  # 딕셔너리 형식으로 저장


    def remove_tasks(self):
        # 첫 번째 작업을 가져와 처리
        if self.task_queue:
            self.task_queue.popleft()  # FIFO 방식으로 처리
            print(f"remove_tasks task [0]")
        else:
            print("No tasks to process.")
    
    '''
    #스토리지 이동 요청이 들어왔을 경우
    def receive_request_callback(self, msg, isFromInterface = True, command = None):   

        self.get_logger().info('Request System: "%s", User Name: "%s"' % (msg.request_system, msg.user_name))   
        ########################################
        if isFromInterface: 
            # 스토리지 이동 요청 처리
            if self.ends_process(msg.user_name):
                if self.task_queue:
                    if self.check_first_requester(msg.user_name):
                        self.remove_tasks() #task que에서 삭제
                    else :
                        return
                else:
                    return
            else:
                self.add_task(msg.user_name, msg)
        else:
            if command is not None:
                msg.user_name = msg.user_name+"_B"
        ##########################################

        print_file_path = msg.file_path
        self.isAvailable = False #    
        
        if self.task_queue:           
            self.get_logger().info(f'task_queue : {len(self.task_queue)}')

        

        ###########################################################################
         # 스토리지 이동 요청지 - 프린터 일 경우 
        ###########################################################################
        if msg.request_system == "print":   
            print('프린터 시스템에서 호출') 

            # 1.스토리지 프린터기로 이동 요청 - ROS 발행 
            send_msg = RobotRequestMoving()             
            send_msg.request_system = msg.request_system  # 메시지 내용              
            send_msg.user_name = msg.user_name # 메시지 내용     
            #send_msg.positions = array('f', PositionDict['print']) # 메시지 내용      
            send_msg.positions = self.get_position_array('print') # 메시지 내용      
                
            self.request_move_publisher.publish(send_msg)           
            self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))     
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")

            # 2.스토리지 도착 완료 - ROS 구독
            self.isArrived = False #스토리지 이동 완료 
            self.get_logger().info(f"Publishing isArrived: '{send_msg}'")
            while not self.isArrived:                       
                # rp.spin_once(self)
                pass

 
            # 3.프린터시스템에 프린터 요청 - ROS 발행         
            send_msg = RobotRecevieMoving()
            send_msg.request_system = msg.request_system  
            send_msg.user_name = msg.user_name # 메시지 내용  
            send_msg.file_path = print_file_path  # 메시지 내용  
           
            self.request_print_publisher.publish(send_msg)
            self.get_logger().info('Publishing: "%s"' % send_msg.file_path)         
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")
           
            # 4.프린터 시스템 인쇄 결과 ( 완료 OR 에러 ) -  ROS 구독
            self.isPrintComplete = False 
            self.result_msg = None
            
            while not self.isPrintComplete:           
                pass  
                # rp.spin_once(self)           
               
            #5. 스토리지 이동 요청 - (5-A)인쇄 성공 : 사용자에게 / (5-B)인쇄 에러 : 복귀
            send_msg = RobotRequestMoving()       
            send_msg.request_system = msg.request_system  # 메시지 내용
         
            #프린터 성공적으로 되었을 경우
            if 'comp' in self.result_msg.data:
                send_msg.user_name = msg.user_name     
            #프린터 실패할경우, 스토리지 복귀
            else:                
                send_msg.user_name = msg.user_name+"_B" 
          
            #send_msg.positions = array('f',PositionDict[msg.user_name])# 메시지 내용   
            send_msg.positions = self.get_position_array(msg.user_name) # 메시지 내용       
            self.request_move_publisher.publish(send_msg)        
            self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")    
            # 6.스토리지 사용자에게  도착 완료 - ROS 구독
            self.isArrived = False #스토리지 이동 완료 
            while not self.isArrived:                  
                # rp.spin_once(self)
                pass
            
           

        ###########################################################################
        # 스토리지 이동 요청지 - (프린터 외) 챗봇
        # 스토리지 이동 목적지 - 사용자 자리 OR 홈 OR 복도
        ###########################################################################
        else:
            # 1.스토리지 이동 요청 - ROS 발행
            send_msg = RobotRequestMoving()
            send_msg.request_system = msg.request_system  # 메시지 내용
            send_msg.user_name = msg.user_name  # 메시지 내용   
            #send_msg.positions = array('f',PositionDict[msg.user_name])# 메시지 내용 - 이동    
            send_msg.positions = self.get_position_array(msg.user_name) # 메시지 내용 
            self.request_move_publisher.publish(send_msg)        
            self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")    
            
            
            # 2.스토리지 도착 완료 - ROS 구독
            self.isArrived = False #스토리지 이동 완료 
            while not self.isArrived:
                # rp.spin_once(self)
                pass
 
        #이동 완료 후 상태값들 초기화
        self.init_vars()

        #마지막 사용자(복귀 명령어는 "")와 시간 저장
        if self.ends_process(msg.user_name):
            self.modify_last_user("")
        else:
            self.modify_last_user(msg.user_name)

    '''

    #스토리지 이동 요청이 들어왔을 경우
    def receive_request_callback(self, msg):   

        self.get_logger().info('Request System: "%s", User Name: "%s"' % (msg.request_system, msg.user_name))          
        self.add_task(msg.user_name, msg)
        if self.task_queue:           
            self.get_logger().info(f'task_queue : {len(self.task_queue)}')

    
    #스토리지 이동 및 처리
    def excute_task(self, msg, command = None):   
        self.get_logger().info('[request_task] Request System: "%s", User Name: "%s"' % (msg.request_system, msg.user_name))   
        print_file_path = msg.file_path
        self.isAvailable = False # 
        ########################################
        
        #대기 사용 시간 완료 후 복귀처리를 위하여
        if command is not None:
            msg.user_name = msg.user_name+"_B"

        if self.task_queue:
            if self.ends_process(msg.user_name):
                if self.check_first_requester(msg.user_name):
                        self.remove_tasks() #task que에서 삭제    
                else :
                    self.isAvailable = True # 
                    return #스토리지를 사용하지 않은 사람이 복귀 명령어를 호출할 경우는 처리X

      
        ##########################################

        ###########################################################################
         # 스토리지 이동 요청지 - 프린터 일 경우 
        ###########################################################################
        if msg.request_system == "print":   
            print('프린터 시스템에서 호출') 

            # 1.스토리지 프린터기로 이동 요청 - ROS 발행 
            send_msg = RobotRequestMoving()             
            send_msg.request_system = msg.request_system  # 메시지 내용              
            send_msg.user_name = msg.user_name # 메시지 내용     
            #send_msg.positions = array('f', PositionDict['print']) # 메시지 내용      
            send_msg.positions = self.get_position_array('print') # 메시지 내용      
                
            self.request_move_publisher.publish(send_msg)           
            self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))     
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")

            # 2.스토리지 도착 완료 - ROS 구독
            self.isArrived = False #스토리지 이동 완료 
            self.get_logger().info(f"Publishing isArrived: '{send_msg}'")
            while not self.isArrived:                       
                # rp.spin_once(self)
                rp.spin_once(self, timeout_sec=0.1)

 
            # 3.프린터시스템에 프린터 요청 - ROS 발행         
            send_msg = RobotRecevieMoving()
            send_msg.request_system = msg.request_system  
            send_msg.user_name = msg.user_name # 메시지 내용  
            send_msg.file_path = print_file_path  # 메시지 내용  
           
            self.request_print_publisher.publish(send_msg)
            self.get_logger().info('Publishing: "%s"' % send_msg.file_path)         
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")
           
            # 4.프린터 시스템 인쇄 결과 ( 완료 OR 에러 ) -  ROS 구독
            self.isPrintComplete = False 
            self.result_msg = None
            
            while not self.isPrintComplete:           
                rp.spin_once(self, timeout_sec=0.1)
                # rp.spin_once(self)           
               
            #5. 스토리지 이동 요청 - (5-A)인쇄 성공 : 사용자에게 / (5-B)인쇄 에러 : 복귀
            send_msg = RobotRequestMoving()       
            send_msg.request_system = msg.request_system  # 메시지 내용
         
            #프린터 성공적으로 되었을 경우
            if 'comp' in self.result_msg.data:
                send_msg.user_name = msg.user_name     
            #프린터 실패할경우, 스토리지 복귀
            else:                
                send_msg.user_name = msg.user_name+"_B" 
          
            #send_msg.positions = array('f',PositionDict[msg.user_name])# 메시지 내용   
            send_msg.positions = self.get_position_array(msg.user_name) # 메시지 내용       
            self.request_move_publisher.publish(send_msg)        
            self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")    
            # 6.스토리지 사용자에게  도착 완료 - ROS 구독
            self.isArrived = False #스토리지 이동 완료 
            while not self.isArrived:                  
                # rp.spin_once(self)
                rp.spin_once(self, timeout_sec=0.1)
            
           

        ###########################################################################
        # 스토리지 이동 요청지 - (프린터 외) 챗봇
        # 스토리지 이동 목적지 - 사용자 자리 OR 홈 OR 복도
        ###########################################################################
        else:
            # 1.스토리지 이동 요청 - ROS 발행
            send_msg = RobotRequestMoving()
            send_msg.request_system = msg.request_system  # 메시지 내용
            send_msg.user_name = msg.user_name  # 메시지 내용   
            #send_msg.positions = array('f',PositionDict[msg.user_name])# 메시지 내용 - 이동    
            send_msg.positions = self.get_position_array(msg.user_name) # 메시지 내용 
            self.request_move_publisher.publish(send_msg)        
            self.get_logger().info('Publising : "%s", User Name: "%s"' % (send_msg.request_system, send_msg.user_name))
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")    
            
            
            # 2.스토리지 도착 완료 - ROS 구독
            self.isArrived = False #스토리지 이동 완료 
            while not self.isArrived:
                # rp.spin_once(self)
                #pass
                rp.spin_once(self, timeout_sec=0.1)
 
        #이동 완료 후 상태값들 초기화
        self.init_vars()

        #마지막 사용자(복귀 명령어는 "")와 시간 저장
        if self.ends_process(msg.user_name):
            self.modify_last_user("")
        else:
            self.modify_last_user(msg.user_name)

    def ends_process(self, variable):
        """주어진 문자열이 '_B'로 끝나는지 확인하는 함수."""
        return variable.endswith('_B')


    def init_vars(self):              
        self.isAvailable = True #스토리지 사용 가능 여부 ( 만약 프린터 시스템에서 사용중일 경우, True 처리 시키기)
        self.isArrived = False #스토리지 이동 완료 
        self.result_msg = None
        self.isPrintComplete = False #프린트완료 여부

    #마지막 사용자 값 저장
    def modify_last_user(self, user):        
        self.last_user_name = user       
        self.last_user_time = time.time() 
        print(f"마지막 사용자 : '{user}'(마지막 이용시간:{self.last_user_time})")   

    # def check_first_requester(self, user):
    #     if self.task_queue:
    #         return next(iter(self.task_queue[0].keys()))  == user.replace('_B', '')  
    #     else:
    #         self.get_logger().info("Task queue is empty.")
    #         return      
        
    def check_first_requester(self, user):
        try:
            if self.task_queue:    
                first_task = self.task_queue[0]  # 첫 번째 큐
                first_task_user = next(iter(first_task))  # 첫 번째 큐의 키 가져오기
                return first_task_user.replace('_B', '') == user.replace('_B', '')
            else:
                self.get_logger().error(f"Unexpected data type in task_queue: {type(self.task_queue[0])}")
                return False
        except IndexError:
            self.get_logger().warning("Task queue is empty.")
            return False
        except Exception as e:
            self.get_logger().error(f"Error in check_first_requester: {e}")
            return False





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
       
class TaskQueManager(Node):
    def __init__(self, taskmanager):
        super().__init__('que_manager')
        self.task_manager = taskmanager
        self.timer = self.create_timer(10, self.task_scheduler)  # 2초마다 타이머 콜백 호출
        
    def task_scheduler(self):
       #현재 스토리지가 실행 중이 아니라면(isAvailable = True)

        if self.task_manager.isAvailable :
            if self.task_manager.task_queue:
                # 첫 번째 큐의 키와 값 가져오기
                first_task = self.task_manager.task_queue[0]  # 첫 번째 큐
                first_task_user = next(iter(first_task))  # 첫 번째 큐의 키 가져오기
                first_task_value = first_task[first_task_user]  # 해당 키에 대한 값 (객체) 가져오기


                #마지막 사용자와 que[0]에 사용자가 일치 하고, 시간이 waiting시간을 초과했다면 종료 , 종료 처리
                if self.task_manager.check_first_requester(self.task_manager.last_user_name):
                    current_time = time.time()
                    # 시간 차이를 계산
                    time_difference = current_time - self.task_manager.last_user_time

                    #스토리지 사용 종료 처리
                    if time_difference > waiting_time:                  
                        #self.task_manager.excute_task(next(iter(self.task_manager.task_queue[0].values())) , "exit")   
                        self.task_manager.excute_task(first_task_value, "exit")   
                        
                    else:
                        pass

                #순차 처리  
                else:
                    #self.task_manager.excute_task(next(iter(self.task_manager.task_queue[0].values())) )   
                    self.task_manager.excute_task(first_task_value)   
            else : 
                self.get_logger().info("Task queue is empty.")
                pass

        else : 
            self.get_logger().info("In use by another process")
            pass
        
          
def main(args=None):
    rp.init(args=args)
    task_manager = TaskManager()
    arrive_subscriber = ArriveSubscriber(task_manager)
    print_subscriber = PrinterSubscriber(task_manager)
    que_manager = TaskQueManager(task_manager)
    executor = MultiThreadedExecutor()
    executor.add_node(task_manager)
    executor.add_node(arrive_subscriber)
    executor.add_node(print_subscriber)
    executor.add_node(que_manager)
  
    try:
        executor.spin()

    finally:
        executor.shutdown()
        task_manager.destroy_node()
        arrive_subscriber.destroy_node()
        print_subscriber.destroy_node()
        que_manager.destroy_node()
        rp.shutdown()     

if __name__ == '__main__':
    main()