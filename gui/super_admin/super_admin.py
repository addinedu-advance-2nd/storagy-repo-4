# paramiko 설치 : pip install paramiko
# pygame 설치 : pip install pygame




import sys
import os
import paramiko
import re
import pygame  # 조이스틱 입력 라이브러리
import time
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt

import subprocess
from PyQt5 import QtWidgets, uic, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QTabWidget, QFrame
from PyQt5.QtCore import QTimer, QStringListModel, Qt, QThread
import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from view_3D import IMUVisualization
from ros_monitor import Ros2MonitorNode
from cam_stream import CamStream
#from storagy_control import StoragyControl
#from gui.super_admin.old.keyboard_control import KeyBoardControl
#from gui.super_admin.old.joystick_control import JoyStickControl
from battery_listener import BatteryListener
from cmd_vel_listener import CmdVelListener
from cam_stream import CameraThread
#from topic_tab import TopicTabView
from motor_state_listener import MotorStateListener
from odom_listener import OdomListener
from cmd_vel_pub import CmdVelPub
from manual_control import ManualControl
from depth_scan_sub import DepthScanSubscriber
from topic_viewer import TopicViewer
from service_viewer import ServiceViewer
from map_topic_listener import MapTopicListener
from tf_listener import TfListener
from depth_cam_listener import DepthCameraVisualization
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from lidar_listener import LidarVisualization


#from gui.super_admin.super_topic import TopicSubscriber

# Domain ID를 10으로 설정
os.environ['ROS_DOMAIN_ID'] = '50'

host = "192.168.0.33"  # 접속할 SSH 서버의 IP 주소나 도메인
username = "storagy"  # SSH 접속에 사용할 사용자 이름
password = "123412"   # SSH 접속에 사용할 비밀번호

# ROS_DOMAIN_ID 환경 변수 읽기
#ros_domain_id = os.getenv("ROS_DOMAIN_ID", "Default Domain ID Not Set")

#print(f"ROS Domain ID: {ros_domain_id}")



class MainWindow(QMainWindow):
    def __init__(self, s_admin, topic_tab_page, nav_tab_page):
        super().__init__()
        #self.ros_node = ros_node

        self.s_admin = s_admin
        self.topic_tab_page = topic_tab_page
        self.nav_tab_page = nav_tab_page


        self.setWindowTitle("ROS2 Monitor with Bandwidth and Frequency")


        self.led_label = self.s_admin.findChild(QLabel, "led_label")

        # 스택 위젯 생성 및 중앙 위젯으로 설정
        self.stacked_widget = QtWidgets.QStackedWidget(self)
        self.setCentralWidget(self.stacked_widget)

        # 탭 위젯
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # 메인 탭
        self.main_tab = QWidget()
        self.tabs.addTab(self.main_tab, "Dashboard")
        self.main_layout = QVBoxLayout()
        self.main_tab.setLayout(self.main_layout)

       # super_admin.ui를 main_tab에 추가
        self.main_layout.addWidget(self.s_admin)

        # 체크박스 시그널 연결
        self.s_admin.dep_cam_view.stateChanged.connect(self.view_3d_dep_checkbox)
        self.s_admin.lidar_view.stateChanged.connect(self.view_3d_lidar_checkbox)
        self.s_admin.rgb_cam_view.stateChanged.connect(self.view_rgb_cam_checkbox)
        self.s_admin.safety_check_button.stateChanged.connect(lambda state: None)
        self.s_admin.keyboard_control_button.toggled.connect(self.keyboard_check_conditions)
        self.s_admin.joystick_control_button.toggled.connect(self.joystick_check_conditions)

        # 창을 최대화된 상태로 표시
        self.showMaximized()


        # 토픽 정보 탭
        tap_list = ["토픽", "서비스", "센서,카메라", "네비게이션", "로봇 상태 및 제어", "지도 및 위치", "기타 이벤트 및 상태"]
 
        self.topic_tab = QWidget()
        self.tabs.addTab(self.topic_tab, "토픽")
        self.topic_layout = QVBoxLayout()
        self.topic_tab.setLayout(self.topic_layout)

        #self.topic_tab_view = TopicTabView(topic_tab_page)  
        self.topic_tab_view = TopicViewer()

        # topic_tab.ui를 main_tab에 추가
        self.topic_layout.addWidget(self.topic_tab_view)

        # 서비스 정보 탭
        
        self.service_tab = QWidget()
        self.tabs.addTab(self.service_tab, "서비스")
        self.service_layout = QVBoxLayout()
        self.service_tab.setLayout(self.service_layout)

        #self.topic_tab_view = TopicTabView(topic_tab_page)  
        self.service_tab_view = ServiceViewer()

        # topic_tab.ui를 main_tab에 추가
        self.service_layout.addWidget(self.service_tab_view)
     
        # 네이게이션 탭 추가
        self.nav_tab = QWidget()
        self.tabs.addTab(self.nav_tab, "네비게이션")
        self.nav_layout = QVBoxLayout()
        self.nav_tab.setLayout(self.nav_layout)

        #self.nav_tap_view = TopicViewer(self.nav_tab_page)  

        # topic_tab.ui를 main_tab에 추가
        #self.nav_layout.addWidget(self.nav_tap_view)

        self.stylesheet_all()


            
        '''    
        # 토픽 정보 탭
        tap_list = ["서비스", "센서,카메라", "네비게이션", "로봇 상태 및 제어", "지도 및 위치", "기타 이벤트 및 상태"]
        for tap in tap_list:
            self.service_tab = QWidget()
            self.tabs.addTab(self.service_tab, tap)
            self.service_layout = QVBoxLayout()
            self.service_tab.setLayout(self.service_layout)
        '''
        #ssh 연결 확인 함수 호출
        #QTimer.singleShot(100, self.check_ssh_connection)  # 10초 후 세 번째 함수 호출
        # SSH 연결 확인을 위한 타이머 설정 (예시: 5초마다 연결 시도)
        self.timer_1 = QTimer(self)
        self.timer_1.timeout.connect(self.check_ssh_connection)
        self.timer_1.start(3000)  # 3초마다 체크



    def stylesheet_all(self):
        
        # QMainWindow 스타일 설정
        self.setStyleSheet("""
            QMainWindow {
                background-color: #212024;  /* 배경색: 짙은 회색 */
                border: 50px solid #212024; /* 배경색: 짙은 회색 */
                
            }
            
        """)

        self.topic_tab.setStyleSheet("background-color: #212024;")
        self.service_tab.setStyleSheet("background-color: #212024;")
        self.nav_tab.setStyleSheet("background-color: #212024;")


        # QFrame 스타일시트 설정
        frame_style = """
            QFrame {
                background-color: #2c2d30;    /* 배경색: dimgrey */
                border-radius: 10px;          /* 테두리 모서리 둥글게: 10px */
            }
        """

        self.s_admin.frame.setStyleSheet(frame_style)
        self.s_admin.battery_frame.setStyleSheet(frame_style)
        self.s_admin.cmd_vel_x_frame.setStyleSheet(frame_style)
        self.s_admin.cmd_vel_z_frame.setStyleSheet(frame_style)
        self.s_admin.odom_frame.setStyleSheet(frame_style)
        self.s_admin.frame_3.setStyleSheet(frame_style)
        self.s_admin.motor_frame.setStyleSheet(frame_style)
        self.s_admin.ssid_frame.setStyleSheet(frame_style)
        self.s_admin.ip_frame.setStyleSheet(frame_style)
        self.s_admin.camera_frame.setStyleSheet(frame_style)
        self.s_admin.camera_frame_2.setStyleSheet(frame_style)
        self.s_admin.camera_frame_3.setStyleSheet(frame_style)
        self.s_admin.tf_frame.setStyleSheet(frame_style)

        
        





        # 글자 색상 설정
        labels = ["label_3", "label_6", "label_7", "checkBox", "keyboard_control_button", "joystick_control_button", "label_2",
                  "odom_2", "odom_3", "odom_4", "motor_state_2", "motor_r", "motor_l", "ssid", "ip" , "label_2", "label_8", "rgb_cam_view",
                  "label_11", "dep_cam_view", "label_12", "lidar_view", "map", "map_1", "map_2"
                  ]

        for label_name in labels:
            label = getattr(self.s_admin, label_name)  # label_name에 해당하는 속성 접근
            label.setStyleSheet("color: white;")  # 스타일 적용



        self.s_admin.safety_check_button.setStyleSheet("color: white;")
        self.s_admin.battery_voltage_2.setStyleSheet("color: white;")
        self.s_admin.cmd_vel_2.setStyleSheet("color: white;")
        self.s_admin.cmd_vel_4.setStyleSheet("color: white;")

        labels_32 = ["odom_pos_x", "odom_pos_y", "odom_pos_z", "odom_ori_x", "odom_ori_y", "odom_ori_z", "odom_ori_w", 
                     "motor_r_A", "motor_r_Nm", "motor_l_A", "motor_l_Nm", "battery_voltage", "cmd_vel_x", "cmd_vel_z",
                     "tf_pos_x", "tf_pos_y", "tf_pos_z", "tf_ori_x", "tf_ori_y", "tf_ori_z", "tf_ori_w"
            ]

        for label_name_32 in labels_32:
            label = getattr(self.s_admin, label_name_32)  # label_name에 해당하는 속성 접근
            label.setStyleSheet("color: #32e6b7;")  # 스타일 적용

        '''
        self.s_admin.odom_pos_x.setStyleSheet("color: #32e6b7;")
        self.s_admin.odom_pos_y.setStyleSheet("color: #32e6b7;")
        self.s_admin.odom_pos_z.setStyleSheet("color: #32e6b7;")
        self.s_admin.odom_ori_x.setStyleSheet("color: #32e6b7;")
        self.s_admin.odom_ori_y.setStyleSheet("color: #32e6b7;")
        self.s_admin.odom_ori_z.setStyleSheet("color: #32e6b7;")
        self.s_admin.odom_ori_w.setStyleSheet("color: #32e6b7;")
        '''







    def keyboard_check_conditions(self):
        # 안전 확인이 되어 있는지 먼저 확인
        if self.s_admin.safety_check_button.isChecked() and self.s_admin.keyboard_control_button.isChecked():
            print("안전 확인 됨, 키보드로 선택됨")
            #self.storagy_control = KeyBoardControl(self.s_admin)
            #self.key_board_control = KeyBoardControl(self.s_admin)
            #self.manual_control = ManualControl(self.s_admin, "keyboard")
            #print(self.key_board_control.send_command)
            self.control_type = "keyboard"        
        else:
            pass

        # manual_control이 실행되고 있으면 종료 후 재 실행
        if hasattr(self, 'manual_control'):
            del self.manual_control
        self.manual_control = ManualControl(self.s_admin, self.control_type)




    def joystick_check_conditions(self):
        if self.s_admin.safety_check_button.isChecked() and self.s_admin.joystick_control_button.isChecked():
            print("안전 확인됨, 조이스틱으로 선택됨")

            #self.storagy_control = JoyStickControl(self.s_admin)
            #self.storagy_control = StoragyControl(self.s_admin, self.joystick)
            self.control_type = "joystick"
        else:
            pass
        # manual_control이 실행되고 있으면 종료 후 재 실행
        if hasattr(self, 'manual_control'):
            del self.manual_control

        self.manual_control = ManualControl(self.s_admin, self.control_type)

    def keyPressEvent(self, event):
        key = event.key()
        print(f"Pressed key: {key}")

        if self.s_admin.safety_check_button.isChecked() and self.s_admin.keyboard_control_button.isChecked():
            # manual_control이 실행되고 있으면 종료 후 재 실행
            if hasattr(self, 'manual_control'):
                del self.manual_control
            #self.keyboard_control = KeyBoardControl(self.s_admin)

            # 키보드 버튼에 따라 매핑된 버튼 클릭
            if key == Qt.Key_W:
                self.s_admin.move_forward_button.click()
                #self.keyboard_control.move_forward()
            elif key == Qt.Key_X:
                self.s_admin.move_backward_button.click()
            elif key == Qt.Key_A:
                self.s_admin.turn_left_button.click()
            elif key == Qt.Key_D:
                self.s_admin.turn_right_button.click()
            elif key == Qt.Key_S:
                self.s_admin.stop_button.click()
            elif key == Qt.Key_Q:
                self.s_admin.rotate_left_button.click()
            elif key == Qt.Key_E:
                self.s_admin.rotate_right_button.click()
            else:
                super().keyPressEvent(event)


    def view_rgb_cam_checkbox(self, state):
        if state == 2:  # 체크박스가 선택되었을 때
            # 카메라 소스 선택
            # 1. 노트북 웹캠 (기본값)
            #camera_source = 0  

            # 2. ESP32-CAM (사용 시 주석 해제)
            camera_source = "http://192.168.0.101/mjpeg/1"

            # 3. ROS 카메라 토픽 `/camera/color/image_raw` (사용 시 OpenCV로 처리 필요, 주석 해제)
            # camera_source = "/camera/color/image_raw"




            # CamStream 인스턴스 생성하여 카메라 스트리밍 시작
            self.cam_stream = CamStream(camera_source, self.s_admin)
            #self.cam_stream = CameraThread(camera_source)
        else:
            # 체크박스가 선택되지 않으면 화면 끄기
            self.clear_camera_stream()

    def clear_camera_stream(self):
        # QLabel의 이미지를 비우는 방법으로 화면 지우기
        self.s_admin.rgb_cam.clear()
        
        # 현재 실행 중인 카메라 스트리밍을 중지할 수 있다면, 중지하는 코드 추가
        if hasattr(self, 'cam_stream'):
            self.cam_stream.stop()
            del self.cam_stream

    

    def view_3d_dep_checkbox(self, state):
        print(state)

        # 'view_3d_dep' 위젯 찾기
        self.view_3d_dep = self.s_admin.findChild(QWidget, "view_3d_dep")

        if self.view_3d_dep is None:
            raise ValueError("Error: 'view_3d_dep' widget not found. Check the UI file and widget name.")

        if state == 2:  # 체크박스 선택 상태
            if self.view_3d_dep.layout() is None:
                # 새로운 레이아웃 설정
                print("Setting new layout for 'view_3d_dep'.")
                self.view_3d_dep_layout = QVBoxLayout(self.view_3d_dep)
                self.view_3d_dep.setLayout(self.view_3d_dep_layout)
            else:
                # 기존 레이아웃 참조
                self.view_3d_dep_layout = self.view_3d_dep.layout()

            # DepthCameraVisualization 인스턴스 추가 (중복 방지)
            if not hasattr(self, 'depth_cam') or self.depth_cam is None:
                self.depth_cam = DepthCameraVisualization(self.s_admin)
                #self.depth_cam = ROS2Thread(DepthCameraVisualization, self.s_admin)
                #self.depth_cam.start()
                #self.view_3d_dep_layout.addWidget(self.depth_cam)

        else:  # 체크박스 해제 상태
            if hasattr(self, 'depth_cam') and self.depth_cam is not None:
                # DepthCameraVisualization 위젯 삭제
                self.view_3d_dep_layout.removeWidget(self.depth_cam)
                self.depth_cam.deleteLater()

                # DepthCameraVisualization 노드 종료
                self.depth_cam.destroy_node()
                self.depth_cam = None

                # 필요 시 레이아웃도 삭제 (레이아웃을 완전히 제거하고 새로 설정 가능)
                self.view_3d_dep.setLayout(None)

    def view_3d_lidar_checkbox(self, state):
        #위젯 찾기
        self.view_3d_lidar = self.s_admin.findChild(QWidget, "view_3d_lidar")

        if self.view_3d_lidar is None:
            raise ValueError("Error: 'view_3d_lidar' widget not found. Check the UI file and widget name.")

        if state == 2:
            # 레이아웃 가져오기 또는 새로 설정
            if self.view_3d_lidar.layout() is None:
                print("Setting new layout for 'view_3d_2'.")
                self.view_3d_dep_layout = QVBoxLayout(self.view_3d_lidar)  # QVBoxLayout 또는 원하는 레이아웃 사용
                self.view_3d_lidar.setLayout(self.view_3d_dep_layout)
            else:
                self.view_3d_dep_layout = self.view_3d_lidar.layout()

            self.view_3d_lidar_layout = self.view_3d_lidar.layout()
            self.imu_widget_lidar = LidarVisualization(self.s_admin)
            #self.imu_widget_lidar = DepthScanSubscriber(self.s_admin, "/scan")
            #self.view_3d_lidar_layout.addWidget(self.imu_widget_lidar)
        else:
            if self.view_3d_dep.layout() is not None:
                # 레이아웃의 모든 위젯 제거
                while self.view_3d_lidar_layout.count():
                    item = self.view_3d_lidar_layout.takeAt(0)
                    widget = item.widget()
                    if widget is not None:
                        self.view_3d_lidar_layout.removeWidget(widget)  # 레이아웃에서 제거
                        widget.deleteLater()  # 메모리에서 삭제
                #print("All widgets removed from 'view_3d_dep'.")




    def list_topics(self):
        # 활성화된 토픽들의 이름과 타입을 가져옴
        topic_names_and_types = self.get_topic_names_and_types()

        # 토픽 목록 출력
        for topic, types in topic_names_and_types:
            self.get_logger().info(f"토픽: {topic}, 타입: {', '.join(types)}")


    
    def update_ui(self):
        # ROS2 노드에서 최신 토픽 데이터 가져오기
        self.ros_node.update_topics()

        # 토픽 목록을 갱신
        # 기존 레이아웃의 항목들을 제거
        while self.topic_layout.count():
            item = self.topic_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()  # 위젯 삭제

        # 새로운 토픽 정보 추가 - 
        topic_list = []
        for topic, types in self.ros_node.topic_list:
            # 토픽 이름과 타입을 QLabel로 표시
            topic_label = QLabel(f"Topic: {topic}")
            self.topic_layout.addWidget(topic_label)
            type_label = QLabel(f"Types: {', '.join(types)}")
            self.topic_layout.addWidget(type_label)

            #print(topic)
            topic_list.append(topic)
 
        topic_list_str = '\n'.join(topic_list)
        self.s_admin.topic_list.setText(topic_list_str)

            
    def check_ssh_connection(self):

        
        try:
            # SSH 클라이언트 객체 생성
            ssh_client = paramiko.SSHClient()

            # 자동으로 비공개 키를 사용할 수 있도록 설정
            ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            # SSH 서버에 연결 (기본 포트 22번 사용)
            ssh_client.connect(hostname=host, username=username, password=password)

            # 연결이 성공하면, 명령어 실행을 통해 접속 확인
            stdin, stdout, stderr = ssh_client.exec_command("echo Connection Successful")

            # 명령어 실행 결과 출력
            result = stdout.read().decode().strip()

            if result == "Connection Successful":
                print("SSH 접속이 성공적으로 이루어졌습니다.")
                self.update_led_status("green", "SSH 접속 성공")

                #로봇의 IP,SSID 화면에 표시
                ssid, ip = self.get_wifi_info()
                # QLabel에 결과 표시
                self.s_admin.ssid.setText(f"SSID : {ssid}")
                self.s_admin.ip.setText(f"IP : {ip}")
            else:
                print("SSH 접속에 문제가 있습니다.")
                self.update_led_status("red", "SSH 접속 실패")

            # 연결 종료
            ssh_client.close()

        except Exception as e:
            print(f"SSH 접속 중 오류 발생: {e}")
            self.update_led_status("red", "SSH 접속 실패")

    def update_led_status(self, status, message):
        """LED 상태에 맞게 레이블 색상과 텍스트 변경"""
        if status == "green":
            self.led_label.setText(f"<b>{message}</b>")  # 텍스트를 굵게 표시
            self.led_label.setStyleSheet("background-color: green; color: white; font-size: 18px;border-radius: 10px")
        elif status == "red":
            self.led_label.setText(f"<b>{message}</b>")  # 텍스트를 굵게 표시
            self.led_label.setStyleSheet("background-color: red; color: white; font-size: 18px;border-radius: 10px;")


    def get_wifi_info(self):
        # SSH 클라이언트 생성
        ssh = paramiko.SSHClient()
        
        # 서버의 SSH 키를 자동으로 추가하도록 설정
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
        # SSH 서버에 연결
        ssh.connect(host, username=username, password=password)
        
        # SSID 읽어오기
        stdin, stdout, stderr = ssh.exec_command('iwgetid -r')
        ssid = stdout.read().decode('utf-8').strip()
        
        # IP 주소 읽어오기
        stdin, stdout, stderr = ssh.exec_command('ip a')
        ip_info = stdout.read().decode('utf-8')

        time.sleep(5)
        # IP 주소 추출 (예: 'inet 192.168.1.100' 부분을 찾기)
        ip_lines = [line for line in ip_info.splitlines() if 'inet ' in line]
        #ip_address = ip_lines[0].split()[1] if ip_lines else 'IP not found'
        #print(ip_address)

        
        ip_addresses = []
        for line in ip_lines:
            # "inet" 뒤에 있는 IP 주소 부분만 추출
            match = re.search(r'inet\s+(\d+\.\d+\.\d+\.\d+)', line)
            if match:
                ip_addresses.append(match.group(1))

        #print(ip_addresses)
        
        # 연결 종료
        ssh.close()

 
        return ssid, ip_addresses[2]
    
# 쓰레드 클래스 정의
class ROS2Thread(QThread):
    def __init__(self, node_class, s_admin):
        super().__init__()
        self.node_class = node_class
        self.s_admin = s_admin

    def run(self):
        #rp.init()
        node = self.node_class(self.s_admin)
        executor = rp.executors.SingleThreadedExecutor()  # 단일 스레드 Executor 사용
        executor.add_node(node)
        while rp.ok():
            executor.spin_once(timeout_sec=0.1)  # spin_once() 사용
        node.destroy_node()
        rp.shutdown()

'''
class DepthCameraVisualization(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'depth_camera_visualization')
        QWidget.__init__(self)

        self.bridge = CvBridge()
        self.depth_data = None

        # ROS2 Subscriber
        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            1
        )

        # PyQt5 UI setup
        self.setWindowTitle("Depth Camera 3D Visualization")
        self.setGeometry(100, 100, 800, 600)

        # Matplotlib setup
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Depth')

        #뷰 각도 변경
        self.ax.view_init(azim=30, elev=15)

        # Canvas for Matplotlib
        self.canvas = FigureCanvasQTAgg(self.figure)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        # Timer for updating the plot
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # Update every 100ms

    def depth_callback(self, msg):
        # Convert ROS2 Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_data = cv_image

    def update_plot(self):
        if self.depth_data is None:
            return

        # Generate 3D points from depth data
        h, w = self.depth_data.shape
        x = np.linspace(-w / 2, w / 2, w)
        y = np.linspace(-h / 2, h / 2, h)
        xv, yv = np.meshgrid(x, y)
        zv = self.depth_data

        self.ax.clear()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Depth')

        # Subsample for faster rendering
        subsample = 10
        self.ax.scatter(xv[::subsample, ::subsample],
                        yv[::subsample, ::subsample],
                        zv[::subsample, ::subsample],
                        c=zv[::subsample, ::subsample], cmap='viridis', s=1)
        
         # Plot red point at the origin
        self.ax.scatter([0], [0], [0], c='red', s=50, label="Origin")
                
        # Add legend
        self.ax.legend()

        # Update plot title and refresh canvas
        self.ax.set_title("3D Camera depth")
        
        self.canvas.draw()
    '''


def main():
    # ROS2 초기화
    rp.init()
 

    # PyQt 애플리케이션 초기화
    app = QApplication(sys.argv)



    # UI 로드 및 스택 위젯 설정
    base_dir = os.path.dirname(os.path.abspath(__file__))
    ui_path = os.path.join(base_dir, "../super_admin/super_admin.ui")  # 파일 위치를 정확히 지정
    s_admin = uic.loadUi(ui_path)

    # UI 로드 및 스택 위젯 설정
    base_dir = os.path.dirname(os.path.abspath(__file__))
    ui_path = os.path.join(base_dir, "../super_admin/topic_tab.ui")  # 파일 위치를 정확히 지정
    topic_tab_page = uic.loadUi(ui_path)

        # UI 로드 및 스택 위젯 설정
    base_dir = os.path.dirname(os.path.abspath(__file__))
    ui_path = os.path.join(base_dir, "../super_admin/nav_tab.ui")  # 파일 위치를 정확히 지정
    nav_tab_page = uic.loadUi(ui_path)

    window = MainWindow(s_admin, topic_tab_page, nav_tab_page)
    window.show()
        

    
    # 각 노드를 별도의 쓰레드에서 실행
    battery_thread = ROS2Thread(BatteryListener, s_admin)
    battery_thread.start()

    cmd_vel_thread = ROS2Thread(CmdVelListener, s_admin)
    cmd_vel_thread.start()

    motor_state_thread = ROS2Thread(MotorStateListener, s_admin)
    motor_state_thread.start()

    odom_thread = ROS2Thread(OdomListener, s_admin)
    odom_thread.start()

    tf_thread = ROS2Thread(TfListener, s_admin)
    tf_thread.start()

    #map_thread = ROS2Thread(MapTopicListener, s_admin)
    #map_thread.start()
    #map_thread = MapTopicListener(s_admin)

    '''
    # 타이머로 100ms마다 ROS2 스핀 갱신
    timer = QTimer()
    timer.timeout.connect(lambda: rp.spin_once(battery_thread, timeout_sec=0.1))
    timer.timeout.connect(lambda: rp.spin_once(cmd_vel_thread, timeout_sec=0.1))
    timer.timeout.connect(lambda: rp.spin_once(motor_state_thread, timeout_sec=0.1))
    timer.timeout.connect(lambda: rp.spin_once(odom_thread, timeout_sec=0.1))
    timer.start(300)  # 300ms마다 ROS2 노드 갱신
    '''
 
    app.exec_()

 
    rp.shutdown()


if __name__ == "__main__":
    main()
