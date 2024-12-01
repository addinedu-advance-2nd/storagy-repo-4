import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Button and Keyboard Mapping")
        
        # 버튼 설정
        self.button1 = QPushButton("Button 1")
        self.button2 = QPushButton("Button 2")
        self.button3 = QPushButton("Button 3")

        # 버튼 클릭 시 동작 설정
        self.button1.clicked.connect(self.on_button1_click)
        self.button2.clicked.connect(self.on_button2_click)
        self.button3.clicked.connect(self.on_button3_click)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.button1)
        layout.addWidget(self.button2)
        layout.addWidget(self.button3)

        # 메인 위젯 설정
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def keyPressEvent(self, event):
        key = event.key()

        # 키보드 버튼에 따라 매핑된 버튼 클릭
        if key == Qt.Key_1:
            self.button1.click()
        elif key == Qt.Key_2:
            self.button2.click()
        elif key == Qt.Key_3:
            self.button3.click()
        else:
            super().keyPressEvent(event)

    # 각 버튼 클릭 시 실행될 함수들
    def on_button1_click(self):
        print("Button 1 clicked")

    def on_button2_click(self):
        print("Button 2 clicked")

    def on_button3_click(self):
        print("Button 3 clicked")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
