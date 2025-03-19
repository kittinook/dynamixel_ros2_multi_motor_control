#!/usr/bin/env python3
import sys
import yaml
from PyQt5 import QtWidgets, QtCore

# Widget สำหรับแสดงและปรับ parameter ของมอเตอร์ (หรือ default parameter)
class MotorControlWidget(QtWidgets.QWidget):
    def __init__(self, motor_id, motor_params):
        super().__init__()
        self.motor_id = motor_id
        self.motor_params = motor_params
        self.init_ui()
    
    def init_ui(self):
        layout = QtWidgets.QFormLayout()
        self.param_fields = {}
        for key, value in self.motor_params.items():
            label = QtWidgets.QLabel(key)
            line_edit = QtWidgets.QLineEdit(str(value))
            self.param_fields[key] = line_edit
            layout.addRow(label, line_edit)
        self.setLayout(layout)
    
    def get_parameters(self):
        updated = {}
        for key, field in self.param_fields.items():
            text = field.text()
            # พยายามแปลงเป็น int หรือ float หากเป็นไปได้
            try:
                updated[key] = int(text)
            except ValueError:
                try:
                    updated[key] = float(text)
                except ValueError:
                    updated[key] = text
        return updated

# Main window ที่มี tab สำหรับควบคุมมอเตอร์และปรับแต่ง parameter
class MainWindow(QtWidgets.QWidget):
    def __init__(self, config_path):
        super().__init__()
        self.config_path = config_path
        self.load_config()
        self.init_ui()

    def load_config(self):
        # อ่านไฟล์ YAML
        with open(self.config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        # ดึงค่าจาก multi_motor_control_node: ros__parameters:
        self.params = self.config.get("multi_motor_control_node", {}).get("ros__parameters", {})
        # อ่าน motor_ids
        self.motor_ids = self.params.get("motor_ids", [])
        # สำหรับแต่ละมอเตอร์ อ่าน parameter เฉพาะในส่วน motor_X
        self.motor_configs = {}
        for motor_id in self.motor_ids:
            key = f"motor_{motor_id}"
            self.motor_configs[motor_id] = self.params.get(key, {})

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()
        self.tabs = QtWidgets.QTabWidget()

        # Tab สำหรับควบคุมมอเตอร์ (ส่ง command)
        self.control_tab = QtWidgets.QWidget()
        control_layout = QtWidgets.QVBoxLayout()
        # สร้างกลุ่มสำหรับแต่ละมอเตอร์
        for motor_id in self.motor_ids:
            group_box = QtWidgets.QGroupBox(f"Motor {motor_id} Control")
            group_layout = QtWidgets.QFormLayout()
            # สร้าง input fields สำหรับ goal_current, goal_velocity, goal_position
            current_edit = QtWidgets.QLineEdit()
            velocity_edit = QtWidgets.QLineEdit()
            position_edit = QtWidgets.QLineEdit()
            # เก็บไว้ใน attribute ของ self แบบ dynamic (หรือใน dict ก็ได้)
            self.__dict__[f"motor_{motor_id}_current"] = current_edit
            self.__dict__[f"motor_{motor_id}_velocity"] = velocity_edit
            self.__dict__[f"motor_{motor_id}_position"] = position_edit
            group_layout.addRow("Goal Current:", current_edit)
            group_layout.addRow("Goal Velocity (rad/s):", velocity_edit)
            group_layout.addRow("Goal Position (rad):", position_edit)
            # ปุ่มส่งคำสั่ง (ในที่นี้แสดงผลใน console เป็นตัวอย่าง)
            send_button = QtWidgets.QPushButton("Send Commands")
            send_button.clicked.connect(lambda checked, mid=motor_id: self.send_commands(mid))
            group_layout.addRow(send_button)
            group_box.setLayout(group_layout)
            control_layout.addWidget(group_box)
        control_layout.addStretch()
        self.control_tab.setLayout(control_layout)
        self.tabs.addTab(self.control_tab, "Motor Control")

        # Tab สำหรับปรับแต่ง Parameter
        self.param_tab = QtWidgets.QWidget()
        param_layout = QtWidgets.QVBoxLayout()
        self.motor_param_widgets = {}
        # สำหรับแต่ละมอเตอร์
        for motor_id in self.motor_ids:
            group_box = QtWidgets.QGroupBox(f"Motor {motor_id} Parameters")
            motor_widget = MotorControlWidget(motor_id, self.motor_configs[motor_id])
            self.motor_param_widgets[motor_id] = motor_widget
            vbox = QtWidgets.QVBoxLayout()
            vbox.addWidget(motor_widget)
            group_box.setLayout(vbox)
            param_layout.addWidget(group_box)
        # เพิ่ม widget สำหรับ default parameter (ค่าที่ขึ้นต้นด้วย default_)
        default_params = {k: v for k, v in self.params.items() if k.startswith("default_")}
        self.default_params_widget = MotorControlWidget("default", default_params)
        default_group = QtWidgets.QGroupBox("Default Parameters")
        default_layout = QtWidgets.QVBoxLayout()
        default_layout.addWidget(self.default_params_widget)
        default_group.setLayout(default_layout)
        param_layout.addWidget(default_group)
        # ปุ่ม Save
        self.save_button = QtWidgets.QPushButton("Save Parameters")
        self.save_button.clicked.connect(self.save_parameters)
        param_layout.addWidget(self.save_button)
        param_layout.addStretch()
        self.param_tab.setLayout(param_layout)
        self.tabs.addTab(self.param_tab, "Parameter Settings")

        layout.addWidget(self.tabs)
        self.setLayout(layout)
        self.setWindowTitle("Motor Control GUI with ROS2 Config")

    def send_commands(self, motor_id):
        # ตัวอย่างฟังก์ชันส่งคำสั่ง (สามารถปรับให้ publish ไปยัง ROS2 node ได้)
        goal_current = self.__dict__[f"motor_{motor_id}_current"].text()
        goal_velocity = self.__dict__[f"motor_{motor_id}_velocity"].text()
        goal_position = self.__dict__[f"motor_{motor_id}_position"].text()
        print(f"Sending commands for Motor {motor_id}:")
        print(f"  Goal Current: {goal_current}")
        print(f"  Goal Velocity: {goal_velocity}")
        print(f"  Goal Position: {goal_position}")
        # สามารถนำไปเชื่อมกับ rclpy publisher ได้ตามต้องการ

    def save_parameters(self):
        # อัปเดต parameter ของแต่ละมอเตอร์จาก GUI
        for motor_id in self.motor_ids:
            updated = self.motor_param_widgets[motor_id].get_parameters()
            self.params[f"motor_{motor_id}"] = updated
        # อัปเดต default parameter
        updated_defaults = self.default_params_widget.get_parameters()
        for key, value in updated_defaults.items():
            self.params[key] = value
        # บันทึกกลับไปยังไฟล์ YAML
        if "multi_motor_control_node" not in self.config:
            self.config["multi_motor_control_node"] = {}
        self.config["multi_motor_control_node"]["ros__parameters"] = self.params
        with open(self.config_path, 'w') as f:
            yaml.dump(self.config, f, default_flow_style=False, sort_keys=False)
        QtWidgets.QMessageBox.information(self, "Saved", "Parameters saved successfully.")

def main():
    app = QtWidgets.QApplication(sys.argv)
    # สมมติว่าไฟล์ config อยู่ที่ "config.yaml"
    config_path = "/home/kittinook/dynamixel_ws/src/dynamixel_ros2_multi_motor_control/config/motor_params.yaml"
    window = MainWindow(config_path)
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
