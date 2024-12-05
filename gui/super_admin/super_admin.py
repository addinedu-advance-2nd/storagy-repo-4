# paramiko 설치 : pip install paramiko
# pygame 설치 : pip install pygame




import sys
import os
import paramiko
import re
import pygame  # 조이스틱 입력 라이브러리

import subprocess
from PyQt5 import QtWidgets, uic, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QTabWidget, QFrame
from PyQt5.QtCore import QTimer, QStringListModel
import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from view_3D import IMUVisualization
from ros_monitor import Ros2MonitorNode
from cam_stream import CamStream
#from storagy_control import StoragyControl
from keyboard_control import KeyBoardControl
from joystick_control import JoyStickControl
from battery_listener import BatteryListener
from cmd_vel_listener import CmdVelListener
from cam_stream import CameraThread
from topic_tab import TopicTabView


#from gui.super_admin.super_topic import TopicSubscriber

host = "192.168.0.4"  # 접속할 SSH 서버의 IP 주소나 도메인
username = "storagy"  # SSH 접속에 사용할 사용자 이름
password = "123412"   # SSH 접속에 사용할 비밀번호



class MainWindow(QMainWindow):
    def __init__(self, s_admin, topic_tab_page):
        super().__init__()
        #self.ros_node = ros_node

        self.s_admin = s_admin
        self.topic_tab_page = topic_tab_page


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
        self.tabs.addTab(self.main_tab, "메인")
        self.main_layout = QVBoxLayout()
        self.main_tab.setLayout(self.main_layout)

       # super_admin.ui를 main_tab에 추가
        self.main_layout.addWidget(self.s_admin)

        # 체크박스 시그널 연결
        self.s_admin.dep_cam_view.stateChanged.connect(self.view_3d_dep_checkbox)
        self.s_admin.lidar_view.stateChanged.connect(self.view_3d_lidar_checkbox)
        self.s_admin.rgb_cam_view.stateChanged.connect(self.view_rgb_cam_checkbox)
        self.s_admin.safety_check_button.stateChanged.connect(lambda state: None)
        self.s_admin.keyboard_control_button.toggled.connect(self.check_conditions)
        self.s_admin.joystick_control_button.toggled.connect(self.check_conditions)

        # 창을 최대화된 상태로 표시
        self.showMaximized()


        # 토픽 정보 탭
        tap_list = ["토픽", "서비스", "센서,카메라", "네비게이션", "로봇 상태 및 제어", "지도 및 위치", "기타 이벤트 및 상태"]
 
        self.topic_tab = QWidget()
        self.tabs.addTab(self.topic_tab, "토픽")
        self.topic_layout = QVBoxLayout()
        self.topic_tab.setLayout(self.topic_layout)


        self.topic_tap_view = TopicTabView(topic_tab_page)  

               # topic_tab.ui를 main_tab에 추가
        self.topic_layout.addWidget(self.topic_tap_view)
     
            
            
        # 토픽 정보 탭
        tap_list = ["서비스", "센서,카메라", "네비게이션", "로봇 상태 및 제어", "지도 및 위치", "기타 이벤트 및 상태"]
        for tap in tap_list:
            self.service_tab = QWidget()
            self.tabs.addTab(self.service_tab, tap)
            self.service_layout = QVBoxLayout()
            self.service_tab.setLayout(self.service_layout)



        # 타이머 설정 (주기적으로 갱신)
        self.timer = QTimer()

        self.timer.start(3000)  # 1초마다 실행

        # SSH 연결 확인을 위한 타이머 설정 (예시: 5초마다 연결 시도)
        #self.timer_1 = QTimer(self)
        #self.timer_1.timeout.connect(self.check_ssh_connection)
        #self.timer_1.start(10000)  # 5초마다 체크


        #print(battery_listener)



    def check_conditions(self):
        # 안전 확인이 되어 있는지 먼저 확인
        if self.s_admin.safety_check_button.isChecked() and self.s_admin.keyboard_control_button.isChecked():
            print("안전 확인 됨, 키보드로 선택됨")
            self.storagy_control = KeyBoardControl(self.s_admin)
        elif self.s_admin.safety_check_button.isChecked() and self.s_admin.joystick_control_button.isChecked():
            print("안전 확인됨, 조이스틱으로 선택됨")
            self.storagy_control = JoyStickControl(self.s_admin)
            #self.storagy_control = StoragyControl(self.s_admin, self.joystick)
        else:
            pass





    def view_rgb_cam_checkbox(self, state):
        if state == 2:  # 체크박스가 선택되었을 때
            # 카메라 소스 선택
            # 1. 노트북 웹캠 (기본값)
            camera_source = 0  

            # 2. ESP32-CAM (사용 시 주석 해제)
            # camera_source = "http://192.168.0.100:81/stream"

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
            self.imu_widget_lidar = IMUVisualization("/scan")
            self.view_3d_lidar_layout.addWidget(self.imu_widget_lidar)
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


    def view_3d_dep_checkbox(self, state):
        print(state)
        #위젯 찾기
        self.view_3d_dep = self.s_admin.findChild(QWidget, "view_3d_dep")

        if self.view_3d_dep is None:
            raise ValueError("Error: 'view_3d_dep' widget not found. Check the UI file and widget name.")

        if state == 2:
            # 레이아웃 가져오기 또는 새로 설정
            if self.view_3d_dep.layout() is None:
                print("Setting new layout for 'view_3d_2'.")
                self.view_3d_dep_layout = QVBoxLayout(self.view_3d_dep)  # QVBoxLayout 또는 원하는 레이아웃 사용
                self.view_3d_dep.setLayout(self.view_3d_dep_layout)
            else:
                self.view_3d_dep_layout = self.view_3d_dep.layout()

            self.view_3d_dep_layout = self.view_3d_dep.layout()
            self.imu_widget_dep = IMUVisualization("/camera/depth/image_raw")
            self.view_3d_dep_layout.addWidget(self.imu_widget_dep)
        else:
            if self.view_3d_dep.layout() is not None:
                # 레이아웃의 모든 위젯 제거
                while self.view_3d_dep_layout.count():
                    item = self.view_3d_dep_layout.takeAt(0)
                    widget = item.widget()
                    if widget is not None:
                        self.view_3d_dep_layout.removeWidget(widget)  # 레이아웃에서 제거
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
            self.led_label.setStyleSheet("background-color: green; color: white; font-size: 18px;")
        elif status == "red":
            self.led_label.setText(f"<b>{message}</b>")  # 텍스트를 굵게 표시
            self.led_label.setStyleSheet("background-color: red; color: white; font-size: 18px;")


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
    '''
class CamStream:
    def __init__(self, s_admin):
        self.s_admin = s_admin
        # 카메라 소스 선택
        # 1. 노트북 웹캠 (기본값)
        camera_source = 0  

        # 2. ESP32-CAM (사용 시 주석 해제)
        # camera_source = "http://192.168.0.100:81/stream"

        # 3. ROS 카메라 토픽 `/camera/color/image_raw` (사용 시 OpenCV로 처리 필요, 주석 해제)
        # camera_source = "/camera/color/image_raw"



        # 카메라 스레드 생성 및 신호 연결
        self.camera_thread = CameraThread(camera_source)
        self.camera_thread.frame_ready.connect(self.update_frame)

    def update_frame(self, frame):
        # QLabel에 프레임 표시
        self.s_admin.rgb_cam.setPixmap(QPixmap.fromImage(frame))
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

    window = MainWindow(s_admin, topic_tab_page)
    window.show()
        


    # 배터리 상태 구독 노드 생성 및 실행
    battery_listener = BatteryListener(s_admin)
    cmd_vel_listener = CmdVelListener(s_admin)
    #self.bat = battery_listener
    print(battery_listener)

    timer = QTimer()
    timer.timeout.connect(lambda: rp.spin_once(battery_listener, timeout_sec=1))
    timer.timeout.connect(lambda: rp.spin_once(cmd_vel_listener, timeout_sec=1))
    timer.start(3000)  # 100ms마다 ROS2 노드 갱신

 
    app.exec_()

 
    rp.shutdown()


if __name__ == "__main__":
    main()
