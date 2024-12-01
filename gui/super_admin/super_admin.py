# paramiko 설치 : pip install paramiko
# inputs 설치 : pip install inputs




import sys
import os
import paramiko
import re
import inputs  # 조이스틱 입력 라이브러리

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


#from gui.super_admin.super_topic import TopicSubscriber

host = "192.168.0.2"  # 접속할 SSH 서버의 IP 주소나 도메인
username = "storagy"  # SSH 접속에 사용할 사용자 이름
password = "123412"   # SSH 접속에 사용할 비밀번호



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        #self.ros_node = ros_node


        self.setWindowTitle("ROS2 Monitor with Bandwidth and Frequency")

        # UI 로드 및 스택 위젯 설정
        base_dir = os.path.dirname(os.path.abspath(__file__))
        ui_path = os.path.join(base_dir, "../super_admin/super_admin.ui")  # 파일 위치를 정확히 지정

        self.s_admin = uic.loadUi(ui_path)

        rp.init()
        ros_node = Ros2MonitorNode(self.s_admin)

        #self.view_3D = IMUVisualization()
        #print(IMUVisualization)

        # PyQt GUI와 ROS2 노드 병렬 실행
        timer = QTimer()
        timer.timeout.connect(lambda: rp.spin_once(ros_node, timeout_sec=0.01))
        #timer.timeout.connect(lambda: rp.spin_once(battery_listener, timeout_sec=0.01))
        timer.start(10)  # 10ms마다 ROS2 노드 갱신
        

        self.led_label = self.s_admin.findChild(QLabel, "led_label")

        # 스택 위젯 생성 및 중앙 위젯으로 설정
        self.stacked_widget = QtWidgets.QStackedWidget(self)
        self.setCentralWidget(self.stacked_widget)

        # 페이지를 스택 위젯에 추가
        #self.stacked_widget.addWidget(self.s_admin)  # 관리자 페이지

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
        #self.showMaximized()

        '''
        # 토픽 정보 탭
        self.topic_tab = QWidget()
        self.tabs.addTab(self.topic_tab, "토픽")
        self.topic_layout = QVBoxLayout()
        self.topic_tab.setLayout(self.topic_layout)

        # 서비스 정보 탭
        self.service_tab = QWidget()
        self.tabs.addTab(self.service_tab, "서비스")
        self.service_layout = QVBoxLayout()
        self.service_tab.setLayout(self.service_layout)
        '''

        tap_list = ["토픽", "서비스", "센서,카메라", "네비게이션", "로봇 상태 및 제어", "지도 및 위치", "기타 이벤트 및 상태"]
        for tap in tap_list:
            self.service_tab = QWidget()
            self.tabs.addTab(self.service_tab, tap)
            self.service_layout = QVBoxLayout()
            self.service_tab.setLayout(self.service_layout)





        # 타이머 설정 (주기적으로 갱신)
        self.timer = QTimer()
        #self.timer.timeout.connect(self.update_ui)
        self.timer.start(3000)  # 1초마다 실행

        # SSH 연결 확인을 위한 타이머 설정 (예시: 5초마다 연결 시도)
        self.timer_1 = QTimer(self)
        self.timer_1.timeout.connect(self.check_ssh_connection)
        #self.timer_1.timeout.connect(self.battery_listener)
        #self.timer_1.timeout.connect(self.cmd_vel_listener)
        #self.timer_1.timeout.connect(self.cmd_vel_nav_listener)
        self.timer_1.start(5000)  # 5초마다 체크

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
            # CamStream 인스턴스 생성하여 카메라 스트리밍 시작
            self.cam_stream = CamStream(self.s_admin)
        else:
            # 체크박스가 선택되지 않으면 화면 끄기
            self.clear_camera_stream()

    def clear_camera_stream(self):
        # QLabel의 이미지를 비우는 방법으로 화면 지우기
        self.s_admin.rgb_cam.clear()
        
        # 현재 실행 중인 카메라 스트리밍을 중지할 수 있다면, 중지하는 코드 추가
        if hasattr(self, 'cam_stream'):
            self.cam_stream.camera_thread.stop()
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

    '''
    def find_topic_label(self, topic):
        # 이미 존재하는 토픽 레이블을 찾음
        for i in range(self.topic_layout.count()):
            widget = self.topic_layout.itemAt(i).widget()
            if isinstance(widget, QLabel) and widget.text().startswith(f"Topic: {topic}"):
                return widget
        return None

    def find_bw_label(self, topic):
        # 이미 존재하는 대역폭 레이블을 찾음
        for i in range(self.topic_layout.count()):
            widget = self.topic_layout.itemAt(i).widget()
            if isinstance(widget, QLabel) and widget.text().startswith(f"  Bandwidth: {topic}"):
                return widget
        return None

    def find_hz_label(self, topic):
        # 이미 존재하는 주파수 레이블을 찾음
        for i in range(self.topic_layout.count()):
            widget = self.topic_layout.itemAt(i).widget()
            if isinstance(widget, QLabel) and widget.text().startswith(f"  Frequency: {topic}"):
                return widget
        return None
    '''
    
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
            # 줄 바꿈 문자로 연결
            #topic_list_str = '\n'.join(topic)

            # QLabel에 텍스트 설정
            #label = QLabel()
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
    

    


def main():
    # ROS2 초기화
    #rp.init()
    #ros_node = Ros2MonitorNode()

    # 배터리 상태 구독 노드 생성 및 실행
    #battery_listener = BatteryListener()

    # PyQt 애플리케이션 초기화
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # PyQt GUI와 ROS2 노드 병렬 실행
    #timer = QTimer()
    #timer.timeout.connect(lambda: rp.spin_once(ros_node, timeout_sec=0.01))
    #timer.timeout.connect(lambda: rp.spin_once(battery_listener, timeout_sec=0.01))
    
    #timer.start(10)  # 10ms마다 ROS2 노드 갱신

    app.exec_()

        # ROS 2 이벤트 루프 실행
    #rp.spin(ros_node)

    # 애플리케이션 종료 시 ROS2 노드 종료
    #battery_listener.destroy_node()
    #ros_node.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    main()
