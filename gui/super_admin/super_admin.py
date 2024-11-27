# paramiko 설치
# pip install paramiko



import sys
import os
import paramiko
import re

import subprocess
from PyQt5 import QtWidgets, uic, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QWidget, QTabWidget
from PyQt5.QtCore import QTimer, QStringListModel
import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
#from gui.super_admin.battery_listener import BatteryListener

host = "192.168.0.4"  # 접속할 SSH 서버의 IP 주소나 도메인
username = "storagy"  # SSH 접속에 사용할 사용자 이름
password = "123412"   # SSH 접속에 사용할 비밀번호


class Ros2MonitorNode(Node):
    def __init__(self):
        super().__init__('ros2_monitor_node')
        self.topic_list = []  # 현재 실행 중인 토픽 목록

    def update_topics(self):
        # 현재 활성화된 토픽 목록 갱신
        self.topic_list = self.get_topic_names_and_types()
        #print(self.topic_list)


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle("ROS2 Monitor with Bandwidth and Frequency")

        # UI 로드 및 스택 위젯 설정
        base_dir = os.path.dirname(os.path.abspath(__file__))
        ui_path = os.path.join(base_dir, "../super_admin/super_admin.ui")  # 파일 위치를 정확히 지정

        self.s_admin = uic.loadUi(ui_path)
        

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

        # 창을 최대화된 상태로 표시
        self.showMaximized()


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

        # 타이머 설정 (주기적으로 갱신)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(3000)  # 1초마다 실행

        # SSH 연결 확인을 위한 타이머 설정 (예시: 5초마다 연결 시도)
        self.timer_1 = QTimer(self)
        self.timer_1.timeout.connect(self.check_ssh_connection)
        self.timer_1.timeout.connect(self.battery_listener)
        self.timer_1.start(5000)  # 5초마다 체크




    '''
    def run_ros2_command(self, command):
        """
        ROS2 명령어를 실행하고 결과를 반환
        """
        try:
            result = subprocess.run(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            if result.returncode != 0:
                return f"Error: {result.stderr.strip()}"
            return result.stdout.strip()
        except Exception as e:
            return f"Error: {e}"
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


    '''
    def print_topic_names(self):
        topics = self.get_topic_names_and_types()
        for name, _ in topics:
            print(name)



        
        # 각 토픽에 대해 대역폭 및 전송 속도 정보 추가
        for topic, _ in self.ros_node.topic_list:
            # 기존 레이블을 업데이트
            topic_label = self.find_topic_label(topic)
            if not topic_label:
                topic_label = QLabel(f"Topic: {topic}")
                self.topic_layout.addWidget(topic_label)

           
            # 대역폭 정보
            bw_command = ["ros2", "topic", "bw", topic]
            bw_result = self.run_ros2_command(bw_command)
            bw_label = self.find_bw_label(topic)
            if not bw_label:
                bw_label = QLabel(f"  Bandwidth: {bw_result}")
                self.topic_layout.addWidget(bw_label)
            else:
                bw_label.setText(f"  Bandwidth: {bw_result}")
            

            # 메시지 전송 속도 정보
            hz_command = ["ros2", "topic", "hz", topic]
            hz_result = self.run_ros2_command(hz_command)
            hz_label = self.find_hz_label(topic)
            if not hz_label:
                hz_label = QLabel(f"  Frequency: {hz_result}")
                self.topic_layout.addWidget(hz_label)
            else:
                hz_label.setText(f"  Frequency: {hz_result}")

         '''
            
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
    

    def battery_listener(self):
        self.subscription = self.create_subscription(
            String,           # 구독할 메시지 타입
            '/battery_voltage',       # 구독할 토픽 이름
            self.listener_callback, # 콜백 함수
            10                       # 큐 크기
        )

    def listener_callback(self, msg):
        # 토픽에서 받은 배터리 상태 메시지를 처리
        print(msg)

        # String 메시지에서 데이터를 추출하고 소수점 두 번째 자리까지 반올림
        try:
            voltage = float(msg.data)  # 문자열을 float로 변환
            voltage_rounded = round(voltage, 2)  # 소수점 두 번째 자리까지 반올림
            self.get_logger().info(f"Battery voltage: {voltage_rounded} V")
            # s_admin에서 QLabel인 volt에 값 출력
            #print(voltage_rounded)
            #self.s_admin.battery_voltage.setText(f"배터리 전압 : {voltage_rounded} V")
        except ValueError:
            self.get_logger().error(f"Received invalid data: {msg.data}")
            # s_admin에서 QLabel인 volt에 값 출력
            self.s_admin.battery_voltage.setText(f"Received invalid data: {msg.data}")
        #self.get_logger().info(f"Battery voltage: {msg.voltage} V")
    '''
class BatteryListener(Node):
    def __init__(self):
        super().__init__('battery_listener')

        # s_admin은 외부에서 넘겨주는 객체 (UI 관련 객체)
        #self.s_admin = s_admin

        # '/battery_voltage' 토픽을 구독
        self.subscription = self.create_subscription(
            String,           # 구독할 메시지 타입
            '/battery_voltage',       # 구독할 토픽 이름
            self.listener_callback, # 콜백 함수
            10                       # 큐 크기
        )

    def listener_callback(self, msg):
        # 토픽에서 받은 배터리 상태 메시지를 처리
        print(msg)

        # String 메시지에서 데이터를 추출하고 소수점 두 번째 자리까지 반올림
        try:
            voltage = float(msg.data)  # 문자열을 float로 변환
            voltage_rounded = round(voltage, 2)  # 소수점 두 번째 자리까지 반올림
            self.get_logger().info(f"Battery voltage: {voltage_rounded} V")
            # s_admin에서 QLabel인 volt에 값 출력
            #print(voltage_rounded)
            #self.s_admin.battery_voltage.setText(f"배터리 전압 : {voltage_rounded} V")
        except ValueError:
            self.get_logger().error(f"Received invalid data: {msg.data}")
            # s_admin에서 QLabel인 volt에 값 출력
            self.s_admin.battery_voltage.setText(f"Received invalid data: {msg.data}")
        #self.get_logger().info(f"Battery voltage: {msg.voltage} V")
'''

            


def main():
    # ROS2 초기화
    rp.init()
    ros_node = Ros2MonitorNode()

    # 배터리 상태 구독 노드 생성 및 실행
    #battery_listener = BatteryListener()

    # PyQt 애플리케이션 초기화
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    # PyQt GUI와 ROS2 노드 병렬 실행
    timer = QTimer()
    timer.timeout.connect(lambda: rp.spin_once(ros_node, timeout_sec=0.01))
    #timer.timeout.connect(lambda: rp.spin_once(battery_listener, timeout_sec=0.01))
    timer.start(10)  # 10ms마다 ROS2 노드 갱신

    app.exec_()

        # ROS 2 이벤트 루프 실행
    #rp.spin(battery_listener)

    # 애플리케이션 종료 시 ROS2 노드 종료
    #battery_listener.destroy_node()
    ros_node.destroy_node()
    #rp.shutdown()


if __name__ == "__main__":
    main()
