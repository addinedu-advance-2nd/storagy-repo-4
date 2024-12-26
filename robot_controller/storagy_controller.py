import rclpy as rp
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from control_msgs.msg import RobotArriveState, RobotRecevieMoving, RobotRequestMoving

import math
import time

from cv_bridge import CvBridge
import cv2
from cv2 import aruco
import numpy as np
import array

import threading

PositionDict = {'init':[0.018, -0.027],
                'printer':[0.018, -0.027],
                'corridor1':[0.018, -0.027],
                'corridor2':[2.33, -0.01],
                'corridor3':[3.93, -4.97],
                'A1':[0.28, 0.79],
                'A2':[0.31, 1.64],
                'B1':[2.45, 0.83],
                'B2':[2.4, 1.56],
                'C1':[4.54, 0.7],
                'C2':[4.56, 1.3]
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
        self.move_subscriber = self.create_subscription(RobotRequestMoving, '/moving_request', self.get_destination, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_arrive_publisher = self.create_publisher(RobotArriveState, '/arrived_receive', 10)
        
        self.goal_handle = None
        # self.move_to_goal('corridor2')
        # self.send_goal([1.0, 0.0])

        self.pid_linear = PID(P=0.5, I=0., D=0.01, max_state=0.15, min_state=-0.15, dt=0.1)
        self.pid_angular = PID(P=1.0, I=0., D=0.0, max_state=0.15, min_state=-0.15, dt=0.1)

        self.goal_x = None
        self.goal_y = None
        self.goal_angle = None

        self.req_sys = ''
        self.user_name = ''
        self.destinations = []

        self.position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.last_position = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # positions = [0.04, 0.62, 2.26, 0.1, 2.41, 2.55]
        # self.destinations = [positions[i:i+2] for i in range(0, len(positions), 2)]
        # self.move_to_goal()

        # self.navigation_to_goal([0.04, 0.62])
        # self.manual_move([[3.96, 0.89]])
        # self.destination = [[0., 1.]]
        # self.move_to_goal()

        self.last_position['x'] = 0.
        self.last_position['x'] = 0.

        # # time.sleep(2)
        # for i in range(20):
        #     self.send_arrive_state(self.req_sys, self.user_name, is_arrived=True)
        #     time.sleep(0.1)

        self.manual_move_timer = self.create_timer(0.1, self.manual_move)
        self.manual_move_trigger = False
        self.goals = [[1.1, 0], [0.9, 0]]
        self.control_mode = 'goal1'
        # self.manual_move()

        

    def get_destination(self, msg):
        print(msg)
        self.req_sys = msg.request_system
        self.user_name = msg.user_name
        positions = msg.positions
        self.destinations = [positions[i:i+2] for i in range(0, len(positions), 2)]
        self.move_to_goal()

    def send_arrive_state(self, request_system, user_name, is_arrived=True):
        msg = RobotArriveState()
        msg.is_arrived = is_arrived
        msg.positions = [self.last_position['x'], self.last_position['y']]
        msg.request_system = request_system
        msg.user_name = user_name
        self.robot_arrive_publisher.publish(msg)
        print('arrived_goal')
    
    def move_to_goal(self):    #움직임 실행 
        if self.destinations:
            print(self.destinations)
            self.navigation_to_goal(self.destinations[0])
     
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

        self.last_position['x'] = self.destinations[0][0]
        self.last_position['x'] = self.destinations[0][0]

        # time.sleep(1)

        print(self.destinations)
        if self.destinations == [array.array('f', (0., 0.))]:
            self.goals = [[1.1, 0], [0.9, 0]]
            self.manual_move_trigger = True
            self.control_mode = 'goal1'
            print(self.manual_move_trigger, self.control_mode)
            return

        self.destinations.pop(0)

        # time.sleep(2)
        if len(self.destinations) == 0:
            for i in range(50):
                self.send_arrive_state(self.req_sys, self.user_name, is_arrived=True)
                time.sleep(0.1)
        else:
            self.move_to_goal()
        

    def feedback_callback(self, feedback):
        # 피드백이 올 때마다 호출되는 콜백
        self.get_logger().info(f'Feedback: {feedback.feedback}')
        
        # print(feedback.feedback.distance_remaining)
        # print(feedback.feedback.navigation_time.sec)

        # 70cm 이내로 들어오면 네비게이션 종료
        if 0.01 < feedback.feedback.distance_remaining < 0.4 and feedback.feedback.navigation_time.sec > 1:
            self.cancel_goal()

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling the goal')
            self._action_client._cancel_goal_async(self.goal_handle)
        else:
            self.get_logger().error('No goal to cancel')
            
    

############## 네비게이션 동작 코드 끝 ################
##################### 직접 조작 코드 ########################

    def manual_move(self):

        if self.manual_move_trigger == False:
            return
        
        global aruco_x, aruco_y, aruco_degree, aruco_detected        

        goal  = [-0.13, 0.0]
        self.goal_x = goal[0]
        self.goal_y = goal[1]
        
        # if aruco_detected == True:
        #     distance_error = self.remain_distance(self.goal_x, self.goal_y, aruco_x, aruco_y)
        #     angle_error = self.distance_angle_diff(self.goal_x, self.goal_y, aruco_x, aruco_y, self.position['theta'])
        #     goal_angle = self.goal_angle_calculator(self.goal_x, self.goal_y, aruco_x, aruco_y)

        #     print(self.goal_x, self.goal_y, aruco_x, aruco_y)
        #     print('aruco_position theta:', self.position['theta'])
        #     print('goal angle: ', goal_angle)
        #     print('angle error:',angle_error)
        # else:

        distance_error = self.remain_distance(self.goal_x, self.goal_y, self.position['x'], self.position['y'])
        angle_error = self.distance_angle_diff(self.goal_x, self.goal_y, self.position['x'], self.position['y'], self.position['theta'])
        goal_angle = self.goal_angle_calculator(self.goal_x, self.goal_y, aruco_x, aruco_y)

        print(self.goal_x, self.goal_y, aruco_x, aruco_y)
        print('amcl_position theta:', self.position['theta'])
        print('goal angle: ', goal_angle)
        print('angle error:',angle_error)
        
        # if self.control_mode == 'goal1':
        #     goal  = self.goals[0]
        #     self.goal_x = goal[0]
        #     self.goal_y = goal[1]


        # if self.control_mode == 'goal2':
        #     goal  = self.goals[1]
        #     self.goal_x = goal[0]
        #     self.goal_y = goal[1]
        

        
        # print('aruco degree:', aruco_degree)


        # print('aruco xy:', aruco_x, aruco_y)
        # print('distance error : ', distance_error)

        # if self.control_mode == 'goal1':
        if distance_error > 0.1:
            self.PID_move(distance_error, angle_error, angle_only=False)
            self.control_mode = 'angle'
        # elif self.control_mode == 'angle':
        #     if abs(self.position['theta']) > 0.3:
        #         self.PID_move(0, self.position['theta'], angle_only=True)
        else:
            print('goal1 succeed')
            self.manual_move_trigger = False
            if self.destinations:
                self.destinations.pop(0)
            for i in range(50):
                self.send_arrive_state(self.req_sys, self.user_name, is_arrived=True)
                time.sleep(0.1)
                

        # elif self.control_mode == 'goal2':
        #     if distance_error > 0.1:
        #         self.PID_move(distance_error, -angle_error, angle_only=False)
        #     else:
        #         self.control_mode = 'goal1'
        #         self.manual_move_trigger = False
        #         print('goal2 succeed')
        #         if self.destinations:
        #             self.destinations.pop(0)
        #         for i in range(50):
        #             self.send_arrive_state(self.req_sys, self.user_name, is_arrived=True)
        #             time.sleep(0.1)

        # elif self.control_mode == 'angle':
        #     if self.normalize_angle(self.position['theta'] - math.pi) > 0.2:
        #         self.PID_move(distance_error, -angle_error, angle_only=True)
        #     else:
        #         self.control_mode = 'goal1'
        #         self.manual_move_trigger = False
        #         self.goals = []
        #         print('angle succeed')
                
            
        #     print('distance error: ', distance_error)
        #     print('angle error: ', angle_error)

        # while distance_error > 0.3:
        #     pose, angle = self.estimate_pose_with_aruco_marker('printer')

        #     distance_error = self.remain_distance(self.goal_x, self.goal_y, pose[0], pose[1])
        #     angle_error = self.distance_angle_diff(self.goal_x, self.goal_y, pose[0], pose[1], angle)
        #     self.PID_move(distance_error, angle_error, angle_only=False)
            
        #     print('distance error: ', distance_error)
        #     print('angle error: ', angle_error)

        #     time.sleep(0.1)

        # while angle_error > 0.3:
        #     pose, angle = self.estimate_pose_with_aruco_marker('printer')
        #     angle_error = self.distance_angle_diff(self.goal_x, self.goal_y, pose[0], pose[1], angle)
        #     self.PID_move(distance_error, angle_error, angle_only=True)

        #     print('distance error: ', distance_error)
        #     print('angle error: ', angle_error)

        #     time.sleep(0.1)
        
        # while distance_error > 0.3:
        #     pose, angle = self.estimate_pose_with_aruco_marker('printer')

        #     distance_error = self.remain_distance(self.goal_x, self.goal_y, pose[0], pose[1])
        #     angle_error = self.distance_angle_diff(self.goal_x, self.goal_y, pose[0], pose[1], angle)
        #     self.PID_move(distance_error, angle_error, angle_only=False)
            
        #     print('distance error: ', distance_error)
        #     print('angle error: ', angle_error)

        #     time.sleep(0.1)

        # if not node_list:
        #     return
        # for node in node_list:
        #     self.set_goal(node)
        #     print('goal:', self.goal_x, self.goal_y)

        #     pose, angle = self.estimate_pose_with_aruco_marker('printer')

        #     distance_error = self.remain_distance(self.goal_x, self.goal_y, pose[0], pose[1])
        #     print(distance_error)

        #     while distance_error > 0.3:
        #         distance_error = self.remain_distance(self.goal_x, self.goal_y, pose[0], pose[1])
        #         angle_error = self.distance_angle_diff(self.goal_x, self.goal_y, pose[0], pose[1], angle)
        #         if angle_error > 0.3:
        #             self.PID_move(distance_error, angle_error, angle_only=True)
        #         else:
        #             self.PID_move(distance_error, angle_error, angle_only=False)
                
        #         print('distance error: ', distance_error)
        #         print('angle error: ', angle_error)

        #         time.sleep(0.2)
        # for i in range(40):
        #     twist = Twist()
        #     twist.linear.x = 0.3
        #     twist.angular.z = 0.0
        #     self.cmd_vel_publisher.publish(twist)
        #     print('cmd vel published')
        #     time.sleep(0.1)

    
    def set_goal(self, node):
        self.goal_x = node[0]
        self.goal_y = node[1]
        # self.goal_angle = node[2]
    
    def remain_distance(self, goal_x, goal_y, position_x, position_y):
        return math.sqrt((goal_x-position_x)**2 + (goal_y-position_y)**2)
    
    def distance_angle_diff(self, goal_x, goal_y, position_x, position_y, position_theta):
        dx = goal_x - position_x
        dy = goal_y - position_y
        angle_error = self.normalize_angle(math.atan2(dy, dx) - position_theta)
        return angle_error
    
    def goal_angle_calculator(self, goal_x, goal_y, position_x, position_y):
        dx = goal_x - position_x
        dy = goal_y - position_y
        angle_error = self.normalize_angle(math.atan2(dy, dx))
        return angle_error

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def PID_move(self, distance_error, angle_error,  angle_only=False):
        twist = Twist()
        angular_speed = self.pid_angular.process(angle_error)
        twist.angular.z = angular_speed
        twist.linear.x = 0.
        if angle_only == False:
            linear_speed = self.pid_linear.process(distance_error)
            twist.linear.x = linear_speed
        
        self.cmd_vel_publisher.publish(twist) 
    
    def estimate_pose_with_aruco_marker(self):
        detected_position = None
        # tvec, rvec = self.detect_aruco_marker()
        # current_pose = aruco_pose[aruco_name] + tvec
        # current_angle = aruco_angle[aruco_name] + rvec
        
        # if detected_position is not None:
        #     if amcl과 거리가 크면 or 각도 차이가 크면:
        #         self.publish_initial_pose(x, y, theta)

        return [aruco_x, aruco_y], aruco_degree
    
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

        print('position :',self.robot.position['x'], self.robot.position['y'], self.robot.position['theta'])
        # print('amcl:', self.robot.position['x'], self.robot.position['y'], self.robot.position['theta'])
        # print(msg.pose.pose.orientation)

    def get_euler_from_quaternion(self, quaternion):
        (x, y, z, w) = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)

        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        # X = math.degrees(math.atan2(t0, t1))
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        # Y = math.degrees(math.asin(t2))
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        # Z = math.degrees(math.atan2(t3, t4))
        Z = math.atan2(t3, t4)

        return X, Y, Z  
        
    # def get_euler_from_quaternion(self, quaternion):       
    #     (x, y, z, w) = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    #     t0 = +2.0 * (w * x + y * z)
    #     t1 = +1.0 - 2.0 * (x * x + y * y)
    #     roll = math.atan2(t0, t1)
    #     t2 = +2.0 * (w * y - z * x)
    #     t2 = +1.0 if t2 > +1.0 else t2
    #     t2 = -1.0 if t2 < -1.0 else t2
    #     pitch = math.asin(t2)
    #     t3 = +2.0 * (w * z + x * y)
    #     t4 = +1.0 - 2.0 * (y * y + z * z)
    #     yaw = math.atan2(t3, t4)
    #     return [yaw, pitch, roll]

calib_data_path = "/home/storagy/Desktop/robopalz/vision/calib_data/MultiMatrix.npz"

with np.load(calib_data_path) as calib_data:
    cam_mat = calib_data["camMatrix"]
    dist_coef = calib_data["distCoef"]
    r_vectors = calib_data["rVector"]
    t_vectors = calib_data["tVector"]

marker_length = 0.05
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()

id = 0
tvec = [0,0,0]
angles = []

aruco_detected = False
aruco_x = 0
aruco_y = 0
aruco_degree = 0

# def rotMat2degree(rotation_matrix):
#     sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
#     singular = sy < 1e-6

#     if not singular:
#         x = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
#         y = math.atan2(-rotation_matrix[2, 0], sy)
#         z = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
#     else:
#         x = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
#         y = math.atan2(-rotation_matrix[2, 0], sy)
#         z = 0

#     x = math.degrees(x)
#     y = math.degrees(y)
#     z = math.degrees(z)
    
#     return x, y, z

def rotMat2radian(rotation_matrix):
    sy = math.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        x = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        y = math.atan2(-rotation_matrix[2, 0], sy)
        z = 0

    return x, y, z

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('Camera_Image_Subscriber')
        

        self.bridge = CvBridge()
        self.cv_image = None


        self.image_subscriber = self.create_subscription(Image, '/camera/color/image_raw', self.camera_callback, 10)

    def camera_callback(self, msg):

        global id, tvec, angles, aruco_x, aruco_y, aruco_degree, aruco_detected

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # cv_image = cv2.resize(cv_image, (320, 240))  
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # corners, ids, rejectedCandidates = self.detector.detectMarkers(gray)
        # print(gray.shape)

        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)   # corners = 2d 픽셀 좌표

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, cam_mat, dist_coef)

            for i in range(len(ids)):
                cv2.drawFrameAxes(gray, cam_mat, dist_coef, rvecs[i], tvecs[i], marker_length)
                cv2.aruco.drawDetectedMarkers(gray, corners, ids)
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                rotation_degree = rotMat2radian(rotation_matrix)

                id = ids[i]
                if id != 2:
                    continue
                tvec = tvecs[i]
                angles = rotation_degree

                camera_rotation_matrix = rotation_matrix.T
                camera_tvec = np.dot(-camera_rotation_matrix, np.matrix(tvec).T)
                camera_tvec = camera_tvec.flatten()

                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = rotation_matrix
                transformation_matrix[:3, 3] = camera_tvec

                # camera2baselink_matrix = [[1,0,0,0],
                #                            [0,1,0,0],
                #                            [0,0,1,-0.198],
                #                            [0,0,0,1]]
                camera2baselink_matrix = [[0,-1,0,0],
                                           [0,0,1,0],
                                           [1,0,0,-0.1980],
                                           [0,0,0,1]]

                
                transformation_matrix = np.dot(transformation_matrix, camera2baselink_matrix)

                # aruco2originbase_matrix = [[0,0,-1,0],
                #                             [-1,0,0,0],
                #                             [0,-1,0,0],
                #                             [0,0,0,1]]
                
                # transformation_matrix = np.dot(transformation_matrix, aruco2originbase_matrix)
                # print(id)
                # print(transformation_matrix)
                # print(transformation_matrix[2, 3])

                aruco_detected = True
                aruco_y = transformation_matrix[0, 3] - 0.07
                aruco_x = transformation_matrix[2, 3] - 1

                # print(aruco_x, aruco_y)

                # print(transformation_matrix)

                rotation_matrix = transformation_matrix[:3, :3]
                rotation_degree = rotMat2radian(rotation_matrix)
                aruco_degree = rotation_degree[2]
                
                

        #         print(f'Rotation : {rotation_matrix}, Translation : {tvecs[i]}')
                # print(f'angle : {rotation_degree}')
                # print(rotation_degree)
        else:
            aruco_detected = False

        # print(aruco_detected)



        
def main(args=None):

    rp.init(args=args)

    robot_controller = RobotController()
    amcl_subscriber = AmclSubscriber(robot_controller)
    Image_subscriber = ImageSubscriber()



    executor = MultiThreadedExecutor()
    executor.add_node(robot_controller)
    executor.add_node(amcl_subscriber)
    executor.add_node(Image_subscriber)
 

    try:
        executor.spin()

    finally:
        executor.shutdown()
        robot_controller.destroy_node()
        amcl_subscriber.destroy_node()
        Image_subscriber.destroy_node()

        rp.shutdown()


if __name__ == '__main__':
    main()