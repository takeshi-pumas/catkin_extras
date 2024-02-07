import subprocess
import os
import signal
from PySide2.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel

class ROSControlWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.navigation_process = None
        self.yolo_process = None
        self.moveit_process = None
        self.moveit_arm_process = None
        self.processes = []

        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        self.navigation_button = QPushButton("Run Navigation")
        self.navigation_button.clicked.connect(self.handle_navigation_button)
        layout.addWidget(self.navigation_button)

        self.yolo_button = QPushButton("Run Object Classification")
        self.yolo_button.clicked.connect(self.handle_yolo_button)
        layout.addWidget(self.yolo_button)

        self.moveit_button = QPushButton("Run Moveit")
        self.moveit_button.clicked.connect(self.handle_moveit_button)
        layout.addWidget(self.moveit_button)

        self.moveit_arm_button = QPushButton("Run Moveit test arm")
        self.moveit_arm_button.clicked.connect(self.handle_moveit_arm_button)
        layout.addWidget(self.moveit_arm_button)

        self.status_label = QLabel("Press the buttons to run ROS nodes.")
        layout.addWidget(self.status_label)

        self.setLayout(layout)
        self.setWindowTitle("ROS Control")

    def run_command(self, command):
        try:
            process = subprocess.Popen(["bash", "-c", f'source ~/.bashrc && et && {command}'], preexec_fn=os.setsid, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return process  # Return the process object
        except FileNotFoundError:
            self.status_label.setText(f"Command '{command.split()[0]}' not found.")
            return None

    def kill_process(self, process):
        if process:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            return None

    def handle_navigation_button(self):
        if self.navigation_process:
            self.navigation_process = self.kill_process(self.navigation_process)
            self.navigation_button.setText("Run Navigation")
            self.status_label.setText("Navigation node stopped.")
        else:
            self.navigation_process = self.run_command('roslaunch nav_pumas navigation_real.launch')
            self.navigation_button.setText("Stop Navigation")
            self.status_label.setText("Navigation node running.")
            self.processes.append(self.navigation_process)

    def handle_yolo_button(self):
        if self.yolo_process:
            self.yolo_process = self.kill_process(self.yolo_process)
            self.yolo_button.setText("Run Object Classification")
            self.status_label.setText("Object Classification node stopped.")
        else:
            self.yolo_process = self.run_command('rosrun object_classification classification_server.py')
            self.yolo_button.setText("Stop Object Classification")
            self.status_label.setText("Object Classification node running.")
            self.processes.append(self.yolo_process)

    def handle_moveit_button(self):
        if self.moveit_process:
            self.moveit_process = self.kill_process(self.moveit_process)
            self.moveit_button.setText("Run Moveit")
            self.status_label.setText("Moveit node stopped.")
        else:
            self.moveit_process = self.run_command('roslaunch hsrb_moveit_config hsrb_demo_with_controller.launch')
            self.moveit_button.setText("Stop Moveit")
            self.status_label.setText("Moveit node running.")
            self.processes.append(self.moveit_process)

    def handle_moveit_arm_button(self):
        if self.moveit_arm_process:
            self.moveit_arm_process = self.kill_process(self.moveit_arm_process)
            self.moveit_arm_button.setText("Run Moveit test arm")
            self.status_label.setText("Moveit test arm node stopped.")
        else:
            self.moveit_arm_process = self.run_command('roslaunch task arm_test.launch')
            self.moveit_arm_button.setText("Stop Moveit test arm")
            self.status_label.setText("Moveit test arm node running.")
            self.processes.append(self.moveit_arm_process)

if __name__ == "__main__":
    app = QApplication([])
    widget = ROSControlWidget()
    widget.show()
    app.exec_()
