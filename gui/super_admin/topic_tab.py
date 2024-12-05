import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QLabel, QComboBox, QWidget


class TopicTabView(QMainWindow):
    def __init__(self, topic_tab_page):
        super().__init__()
        self.setWindowTitle("QComboBox Example")

        self.topic_tab_page = topic_tab_page

        # 메인 위젯과 레이아웃
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        # QLabel 생성
        #self.result_label = QLabel("결과가 여기에 표시됩니다.")
        #self.layout.addWidget(self.result_label)

        # QComboBox 생성
        #self.combo_box = QComboBox()
        self.topic_tab_page.topic_combo_box.addItems(["배터리 잔량", "항목2", "항목3"])
        self.topic_tab_page.topic_combo_box.currentIndexChanged.connect(self.update_label)
        self.layout.addWidget(self.topic_tab_page)

    def update_label(self):
        # QComboBox의 현재 선택된 항목 가져오기
        selected_item = self.topic_tab_page.topic_combo_box.currentText()
        
        # 선택된 항목에 따라 다른 함수 실행
        result = self.execute_function(selected_item)
        
        # QLabel에 결과 표시
        self.topic_tab_page.data_label.setText(result)

    def execute_function(self, item):
        # 선택된 항목에 따라 다른 결과 반환
        if item == "배터리 잔량":
            return "/battery_voltage"
        elif item == "항목2":
            return "항목2가 선택되었습니다!"
        elif item == "항목3":
            return "항목3이 선택되었습니다!"
        else:
            return "알 수 없는 항목입니다."


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TopicTabView()
    window.resize(300, 150)
    window.show()
    sys.exit(app.exec_())
