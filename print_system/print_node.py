import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from control_msgs.msg import RobotRecevieMoving, AlertToChat
from print import Printer  # printer 모듈 가져오기

class PrintNode(Node):
    def __init__(self):
        super().__init__('print_node')  # 노드 이름
        self.isActive = False
        self.user_name = ""
        self.file_path = ""
     
        # 발행 - 스토리지 이동 요청
        self.publisher_request_move= self.create_publisher(RobotRecevieMoving, '/moving_receive', 10) 
        
        # 구독 - 인쇄 시작 요청( = 스토리지 프린터기로 도착 했을때 TaskManger에서 발행) 
        self.subscription_manager = self.create_subscription(RobotRecevieMoving,'/print_request',  self.listener_print_callback, 10)  # 큐 크기       
        self.subscription_manager

        # 발행 - 프린터 인쇄 완료
        self.publisher_print_complete = self.create_publisher(String, '/print_complete', 10) 

        # 발행 - 프린터 오류 ( = 디스코봇 채팅서버로 )
        self.publisher_print_error = self.create_publisher(AlertToChat, '/alert_chat', 10) 

              

    def send_request_print_callback(self, user_name, file_path):       
        self.isActive = True
        self.user_name = user_name
        self.file_path = file_path

        # 1. 스토리지 프런터기로 이동 요청 - ROS 발행
        msg = RobotRecevieMoving()
        msg.request_system = 'print'  # 메시지 내용
        msg.user_name = user_name  # 메시지 내용
        msg.file_path= file_path
        self.publisher_request_move.publish(msg)        
        self.get_logger().info('Publishing: "%s"' % msg.request_system)

        

    def send_print_callback(self, result_msg):       
        msg = String()
        msg.data = result_msg  # 메시지 내용  
        self.publisher_print.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


    #스토리지 프린터기 앞 도착 완료(출력 요청) - ROS구독 (매니저->프린터시스템)
    def listener_print_callback(self, msg):
         # 구독한 메시지를 출력        
        self.get_logger().info('I heard : "%s", User Name: "%s", file_path: %s' % (msg.request_system, msg.user_name, msg.file_path))

        # 1. 프린터 인쇄 요청
        printer = Printer()# 프린터 인스턴스 생성
        result_msg = printer.print_file(msg.file_path)  # 프린터 모듈 사용
        

        # 4. 프린터 인쇄 결과 ROS 발행 - ROS 발행
        send_msg = String()
        if '완료' in result_msg :            
            send_msg.data = "complete" 
            # 5A - TaskManger에 전달 인쇄 완료 전달
            self.publisher_print_complete.publish(send_msg)
            self.get_logger().info('Publishing publisher_print_complete: "%s"' % send_msg.data)   
            self.get_logger().info(f"Publishing Detail: '{send_msg}'")
        else: 
            send_msg.data = "error" 
            # 5B-1. TaskManger에 전달 인쇄 오류 전달
            self.publisher_print_complete.publish(send_msg)            
            self.get_logger().info(f"Publishing publisher_print_complete Detail: '{send_msg}'")

            # 5B-2. 디스코봇 채팅봇에 오류 사항 전달
            send_msg = AlertToChat()
            send_msg.user_name = msg.user_name
            send_msg.user_id = ""  # 메시지 내용     
            self.publisher_print_error.publish(send_msg)           
            self.get_logger().info(f"Publishing publisher_print_error Detail: '{send_msg}'")
      
        self.isActive = False   

        

def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    print_node = PrintNode()

    try:
        rclpy.spin(print_node)  # 노드 실행
    except KeyboardInterrupt:
        pass
    finally:
        print_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()