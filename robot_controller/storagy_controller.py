import rclpy as rp
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

# from control_msgs.msg import RobotArriveState, RobotRecevieMoving, RobotRequestMoving

import math
import time

from cv_bridge import CvBridge
import cv2
from cv2 import aruco

PositionDict = {'init':[0, 0],
                'printer':[0, 1],
                'corridor1':[0.04, 0.62],
                'corridor2':[2.26, 0.79],
                'corridor3':[3.96, 0.89],
                'A1':[0.26, 1.97],
                'A2':[0.33, 2.64],
                'B1':[2.39, 1.78],
                'B2':[2.41, 2.55],
                'C1':[4.47, 1.68],
                'C2':[4.54, 2.16]
                }

class PID:
    def __init__(self, P, I, D, max_state, min_state, dt):
        self.P = P
        self.I = I
        self.D = D
        self.max_state = max_state
        self.min_state = min_state
        self.pre_state = 0.0
        self.dt = dt
        self.integrated_state = 0.0
        self.pre_time = rp.clock.Clock().now().to_msg().sec

    def process(self, state):
        current_time = rp.clock.Clock().now().to_msg().sec
        dt = current_time - self.pre_time

        if dt == 0.:
            state_D = 0.
        else:
            state_D = (state - self.pre_state) / dt

        state_I = self.integrated_state + state * dt

        out = self.P * state + self.D * state_D + self.I * state_I

        if out > self.max_state:
            out = self.max_state
        elif out < self.min_state:
            out = self.min_state

        self.pre_state = state
        self.integrated_state = state_I
        self.pre_time = current_time

        return out
    


class RobotController(Node):
    def __init__(self):
        super().__init__('RobotController')
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self.move_subscriber = self.create_subscription(RobotRequestMoving, '/moving_request', self.get_destination, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.robot_arrive_publisher = self.create_publisher(RobotArriveState, '/arrived_receive', 10)
        # self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        
        self.goal_handle = None
        # self.move_to_goal('corridor2')
        # self.send_goal([1.0, 0.0])

        self.pid_linear = PID(P=0.3, I=0., D=0.01, max_state=0.3, min_state=-0.3, dt=0.1)
        self.pid_angular = PID(P=0.6, I=0., D=0.0, max_state=0.3, min_state=-0.3, dt=0.1)

        self.goal_x = None
        self.goal_y = None
        self.goal_angle = None

        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # self.navigation_to_goal([0.04, 0.62])
        # self.manual_move([[3.96, 0.89]])
        self.move_to_goal([[0.04, 0.62],[2.26, 0.79], [2.41, 2.55]])

        

    def get_destination(self, msg):
        req_sys = msg.request_system
        user_name = msg.user_name
        self.destination = msg.positions

        self.move_to_goal()

        # self.send_arrive_state(req_sys, user_name, is_arrived=True)

    # def send_arrive_state(self, request_system, user_name, is_arrived=True):
    #     msg = RobotArriveState()
    #     msg.is_arrived = is_arrived
    #     msg.positions = [self.position['x'], self.position['y']]
    #     msg.request_system = request_system
    #     msg.user_name = user_name
    #     self.robot_arrive_publisher.publish(msg)
    #     print('arrived_goal')
    
    def move_to_goal(self):    #움직임 실행 
        if self.destination:
            self.navigation_to_goal(self.destination[0])
            self.destination.pop(0)
        # while self.navigation_moving:
        #     pass
        #     # self.get_logger().info("waiting for navigation to finish...")
        # # self.manual_move(self.destination[1:])
        
        # self.navigation_moving = True
        # self.navigation_to_goal(destination[1])
        # while self.navigation_moving:
        #     # self.get_logger().info("waiting for navigation to finish...")
        #     pass
        # self.navigation_moving = True
        # self.navigation_to_goal(destination[2])
        # while self.navigation_moving:
        #     # self.get_logger().info("waiting for navigation to finish...")
        #     pass
     
    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Robot stopped.")

##################### 네비게이션 패키지 동작 코드###############

    def navigation_to_goal(self, destination):  
        #destination = PositionDict[destination]  # destination이 목표 지점 문자열일경우
        self.send_goal(destination)

    def send_goal(self, position):
        # 목표 위치 설정 (예: (x=2, y=2))
        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = position[0]
        goal.pose.pose.position.y = position[1]
        # goal.pose.pose.orientation.w = 1.0  # 로봇의 방향 설정 (회전 없이 직진)
        
        self._action_client.wait_for_server()

        # 액션 서버로 목표를 전송
        self._send_goal_future = self._action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # 액션 서버의 목표 응답을 처리
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            self.get_logger().info('Goal accepted, waiting for result...')
            self._result_future = future.result().get_result_async()
            self._result_future.add_done_callback(self.result_callback)
        else:
            self.get_logger().error('Goal rejected')

    def result_callback(self, future):
        # 결과가 완료되면 호출되는 콜백
        result = future.result().result
        self.navigation_moving = False
        if result:
            self.get_logger().info('Goal succeeded!')
        
        self.move_to_goal()

        # 여기 도착 완료 토픽 발행, 아이디, 주문 같이 반환

    def feedback_callback(self, feedback):
        # 피드백이 올 때마다 호출되는 콜백
        self.get_logger().info(f'Feedback: {feedback.feedback}')
        
        # print(feedback.feedback.distance_remaining)
        # print(feedback.feedback.navigation_time.sec)

        # 70cm 이내로 들어오면 네비게이션 종료
        if 0.01 < feedback.feedback.distance_remaining < 0.7 and feedback.feedback.navigation_time.sec > 1:
            self.cancel_goal()

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling the goal')
            self._action_client._cancel_goal_async(self.goal_handle)
        else:
            self.get_logger().error('No goal to cancel')
            
    

############## 네비게이션 동작 코드 끝 ################
##################### 직접 조작 코드 ########################

    def manual_move(self, node_list):
        if not node_list:
            return
        for node in node_list:
            self.set_goal(node)
            print('goal:', self.goal_x, self.goal_y)

            distnace_error = self.remain_distance()
            print(distnace_error)

            while distnace_error > 0.3:
                distnace_error = self.remain_distance()
                angle_error = self.distance_angle_diff()
                if angle_error > 0.3:
                    self.PID_move(distnace_error, angle_error, angle_only=True)
                else:
                    self.PID_move(distnace_error, angle_error, angle_only=False)
                
                print('distance error: ', distnace_error)
                print('angle error: ', angle_error)

                time.sleep(0.2)
        # for i in range(10):
        #     twist = Twist()
        #     twist.linear.x = -0.3
        #     twist.angular.z = 0.0
        #     self.cmd_vel_publisher.publish(twist)
        #     print('cmd vel published')
        #     time.sleep(0.2)

    
    def set_goal(self, node):
        self.goal_x = node[0]
        self.goal_y = node[1]
        # self.goal_angle = node[2]
    
    def remain_distance(self):
        return math.sqrt((self.goal_x-self.position['x'])**2 + (self.goal_y-self.position['y'])**2)
    
    def distance_angle_diff(self):
        dx = self.goal_x - self.position['x']
        dy = self.goal_y - self.position['y']
        angle_error = self.normalize_angle(math.atan2(dy, dx) - self.position['theta'])
        return angle_error

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def PID_move(self, distnace_error, angle_error,  angle_only=False):
        pose = self.estimate_pose_with_aruco_marker()
        if pose is None:
            pose = self.position # 네비게이션 amcl pose

        twist = Twist()
        angular_speed = self.pid_angular.process(angle_error)
        twist.angular.z = angular_speed
        twist.linear.x = 0.
        if angle_only == False:
            linear_speed = self.pid_linear.process(distnace_error)
            twist.linear.x = linear_speed
        
        self.cmd_vel_publisher.publish(twist) 
    
    def estimate_pose_with_aruco_marker(self):
        detected_position = None
        # detected_position = self.detect_aruco_marker()
        # if detected_position is not None:
        #     if amcl과 거리가 크면 or 각도 차이가 크면:
        #         self.publish_initial_pose(x, y, theta)

        return detected_position
    
    # def rotate_to_goal(self):                 # 이거 이제 안씀
    #     dx = self.goal_x - self.position['x']
    #     dy = self.goal_y - self.position['y']
    #     target_angle = math.atan2(dy, dx)
    #     angle_error = self.normalize_angle(target_angle - self.position['theta'])
    #     angular_speed = self.pid_angular.process(angle_error)
    #     twist = Twist()
    #     twist.angular.z = angular_speed
    #     self.cmd_vel_publisher.publish(twist)



####################### 직접 조작 끝 #######################

class AmclSubscriber(Node):
    def __init__(self, robot):
        super().__init__('amcl_sub_node')

        self.amcl_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        self.robot = robot
    
    def amcl_pose_callback(self, msg):
        # print(msg)
        self.robot.position['x'] = msg.pose.pose.position.x
        self.robot.position['y'] = msg.pose.pose.position.y
        _, _, self.robot.position['theta'] = self.get_euler_from_quaternion(msg.pose.pose.orientation)
        print('amcl:', self.robot.position['x'], self.robot.position['y'], self.robot.position['theta'])
        
    def get_euler_from_quaternion(self, quaternion):       
        (x, y, z, w) = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [yaw, pitch, roll]

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('Camera_Image_Subscriber')
#         self.image_subscriber = self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, 10)

#         self.bridge = CvBridge()

#         aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
#         parameters = aruco.DetectorParameters()
#         self.detector = aruco.ArucoDetector(aruco_dict, parameters)

#     def camera_callback(self, msg):
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

#         corners, ids, rejectedCandidates = self.detector.detectMarkers(gray)
#         print(cv_image.shape)


        
def main(args=None):

    rp.init(args=args)
    robot_controller = RobotController()
    amcl_subscriber = AmclSubscriber(robot_controller)
    # camera_subscriber = ImageSubscriber()
    

    executor = MultiThreadedExecutor()
    executor.add_node(robot_controller)
    executor.add_node(amcl_subscriber)
    # executor.add_node(camera_subscriber)
 

    try:
        executor.spin()

    finally:
        executor.shutdown()
        robot_controller.destroy_node()

        rp.shutdown()


if __name__ == '__main__':
    main()