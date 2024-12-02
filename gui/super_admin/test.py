import sys
import pygame
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt5.QtCore import Qt, QTimer


class JoystickWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Joystick Ball Control")
        self.setGeometry(100, 100, 800, 600)

        # Ball position and radius
        self.ball_x = self.width() // 2
        self.ball_y = self.height() // 2
        self.ball_radius = 20
        self.ball_speed = 5

        # Label for displaying the ball
        self.ball_label = QLabel(self)
        self.ball_label.setStyleSheet("background-color: red; border-radius: 20px;")
        self.ball_label.setGeometry(
            self.ball_x - self.ball_radius,
            self.ball_y - self.ball_radius,
            self.ball_radius * 2,
            self.ball_radius * 2,
        )

        # Timer for updating joystick input
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_position)
        self.timer.start(16)  # ~60 FPS

        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            self.joystick = None
            print("No joystick detected.")

    def update_position(self):
        if self.joystick:
            pygame.event.pump()

            # Read joystick axes
            axis_x = self.joystick.get_axis(0)  # X-axis
            axis_y = self.joystick.get_axis(1)  # Y-axis

            # Update ball position based on joystick input
            self.ball_x += int(axis_x * self.ball_speed)
            self.ball_y += int(axis_y * self.ball_speed)

            # Boundary checks
            self.ball_x = max(self.ball_radius, min(self.width() - self.ball_radius, self.ball_x))
            self.ball_y = max(self.ball_radius, min(self.height() - self.ball_radius, self.ball_y))

            # Update ball label position
            self.ball_label.move(self.ball_x - self.ball_radius, self.ball_y - self.ball_radius)

    def closeEvent(self, event):
        pygame.quit()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = JoystickWindow()
    window.show()
    sys.exit(app.exec_())
