#!/usr/bin/env python3
import sys
import time
import threading
import yaml

from PyQt5 import QtWidgets, QtCore

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import JointState

# สำหรับ plot ด้วย Matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


###############################################################################
# ROS2 Interface Node
###############################################################################
class RosInterface(Node):
    def __init__(self):
        super().__init__('gui_ros_interface')
        # Publishers สำหรับแต่ละ parameter (ตัวอย่างใช้ motor ID 12)
        self.goal_current_pub = self.create_publisher(Int16, 'motor_12/goal_current', 10)
        self.goal_velocity_pub = self.create_publisher(Float32, 'motor_12/goal_velocity', 10)
        self.goal_position_pub = self.create_publisher(Float32, 'motor_12/goal_position', 10)
        # Subscriber สำหรับ joint state feedback
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.latest_joint_state = None

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def publish_goal_current(self, value: int):
        msg = Int16()
        msg.data = value
        self.goal_current_pub.publish(msg)
        self.get_logger().info(f"Published goal current: {value}")

    def publish_goal_velocity(self, value: float):
        msg = Float32()
        msg.data = value
        self.goal_velocity_pub.publish(msg)
        self.get_logger().info(f"Published goal velocity: {value}")

    def publish_goal_position(self, value: float):
        msg = Float32()
        msg.data = value
        self.goal_position_pub.publish(msg)
        self.get_logger().info(f"Published goal position: {value}")


###############################################################################
# Widget สำหรับแสดง/ปรับ Parameter ของมอเตอร์
###############################################################################
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


###############################################################################
# Matplotlib Canvas สำหรับ Plot Feedback vs Commanded
###############################################################################
class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=6, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        # สร้าง 3 subplot สำหรับ current, velocity, position
        self.ax_current = self.fig.add_subplot(311)
        self.ax_velocity = self.fig.add_subplot(312)
        self.ax_position = self.fig.add_subplot(313)
        super().__init__(self.fig)
        self.setParent(parent)
        self.fig.tight_layout()

    def plot_data(self, time_data, cmd_current, fb_current,
                        cmd_velocity, fb_velocity,
                        cmd_position, fb_position):
        # ล้างกราฟเก่า
        self.ax_current.cla()
        self.ax_velocity.cla()
        self.ax_position.cla()

        # Plot current
        self.ax_current.plot(time_data, cmd_current, label='Commanded')
        self.ax_current.plot(time_data, fb_current, label='Feedback')
        self.ax_current.set_title("Current")
        self.ax_current.legend()

        # Plot velocity
        self.ax_velocity.plot(time_data, cmd_velocity, label='Commanded')
        self.ax_velocity.plot(time_data, fb_velocity, label='Feedback')
        self.ax_velocity.set_title("Velocity (rad/s)")
        self.ax_velocity.legend()

        # Plot position
        self.ax_position.plot(time_data, cmd_position, label='Commanded')
        self.ax_position.plot(time_data, fb_position, label='Feedback')
        self.ax_position.set_title("Position (rad or deg)")
        self.ax_position.legend()

        self.draw()


###############################################################################
# Main Window: รวม GUI ทั้งการควบคุม, ปรับแต่ง Parameter และ Plot
###############################################################################
class MainWindow(QtWidgets.QWidget):
    def __init__(self, config_path, ros_interface: RosInterface):
        super().__init__()
        self.config_path = config_path
        self.ros_interface = ros_interface
        # สำหรับเก็บข้อมูล plot: key เป็น motor_id
        # แต่ละ motor มี: time, current_cmd, current_fb, velocity_cmd, velocity_fb, position_cmd, position_fb
        self.plot_data = {}
        self.load_config()
        self.init_ui()
        # Timer สำหรับอัปเดต Plot ทุกๆ 500ms
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(500)

    def load_config(self):
        # อ่านไฟล์ YAML
        with open(self.config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        self.params = self.config.get("multi_motor_control_node", {}).get("ros__parameters", {})
        self.motor_ids = self.params.get("motor_ids", [])
        self.motor_configs = {}
        for motor_id in self.motor_ids:
            key = f"motor_{motor_id}"
            self.motor_configs[motor_id] = self.params.get(key, {})
            # Initialize plot data สำหรับ motor นี้
            self.plot_data[motor_id] = {
                'time': [],
                'current_cmd': [],
                'current_fb': [],
                'velocity_cmd': [],
                'velocity_fb': [],
                'position_cmd': [],
                'position_fb': []
            }

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
            # เก็บไว้ใน attribute ของ self แบบ dynamic
            self.__dict__[f"motor_{motor_id}_current"] = current_edit
            self.__dict__[f"motor_{motor_id}_velocity"] = velocity_edit
            self.__dict__[f"motor_{motor_id}_position"] = position_edit
            group_layout.addRow("Goal Current:", current_edit)
            group_layout.addRow("Goal Velocity (rad/s):", velocity_edit)
            group_layout.addRow("Goal Position (rad):", position_edit)
            # ปุ่มส่งคำสั่ง
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
        for motor_id in self.motor_ids:
            group_box = QtWidgets.QGroupBox(f"Motor {motor_id} Parameters")
            motor_widget = MotorControlWidget(motor_id, self.motor_configs[motor_id])
            self.motor_param_widgets[motor_id] = motor_widget
            box_layout = QtWidgets.QVBoxLayout()
            box_layout.addWidget(motor_widget)
            group_box.setLayout(box_layout)
            param_layout.addWidget(group_box)
        # สำหรับ default parameter
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

        # Tab สำหรับ Plot Feedback vs Commanded
        self.plot_tab = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout()
        # Dropdown สำหรับเลือก motor
        self.motor_selector = QtWidgets.QComboBox()
        for motor_id in self.motor_ids:
            self.motor_selector.addItem(f"Motor {motor_id}", motor_id)
        plot_layout.addWidget(self.motor_selector)
        # Matplotlib canvas
        self.plot_canvas = PlotCanvas(self, width=5, height=6, dpi=100)
        plot_layout.addWidget(self.plot_canvas)
        self.plot_tab.setLayout(plot_layout)
        self.tabs.addTab(self.plot_tab, "Feedback Plot")

        layout.addWidget(self.tabs)
        self.setLayout(layout)
        self.setWindowTitle("Motor Control GUI with ROS2 Config and Plot")

    def send_commands(self, motor_id):
        # ดึงค่าจาก input field สำหรับ motor ที่เลือก
        try:
            goal_current = int(self.__dict__[f"motor_{motor_id}_current"].text())
        except ValueError:
            goal_current = 0
        try:
            goal_velocity = float(self.__dict__[f"motor_{motor_id}_velocity"].text())
        except ValueError:
            goal_velocity = 0.0
        try:
            goal_position = float(self.__dict__[f"motor_{motor_id}_position"].text())
        except ValueError:
            goal_position = 0.0

        print(f"Sending commands for Motor {motor_id}:")
        print(f"  Goal Current: {goal_current}")
        print(f"  Goal Velocity: {goal_velocity}")
        print(f"  Goal Position: {goal_position}")

        # ตัวอย่าง: ส่ง command ผ่าน ROS (สำหรับ motor 12 เท่านั้นในที่นี้)
        # หากต้องการให้ส่งสำหรับ motor อื่น ๆ ให้ปรับ publisher ให้เหมาะสม
        if motor_id == 12:
            self.ros_interface.publish_goal_current(goal_current)
            self.ros_interface.publish_goal_velocity(goal_velocity)
            self.ros_interface.publish_goal_position(goal_position)

        # บันทึกค่าที่สั่งใน time series สำหรับ plot
        t = time.time()
        data = self.plot_data[motor_id]
        data['time'].append(t)
        data['current_cmd'].append(goal_current)
        data['velocity_cmd'].append(goal_velocity)
        data['position_cmd'].append(goal_position)
        # สำหรับ feedback ให้เติมค่าในตอนที่ update_plot() (ถ้ายังไม่มีค่าให้ใส่ None)
        if len(data['current_fb']) < len(data['time']):
            data['current_fb'].append(0)
            data['velocity_fb'].append(0)
            data['position_fb'].append(0)

    def update_plot(self):
        # เลือก motor จาก dropdown
        motor_id = self.motor_selector.currentData()
        data = self.plot_data[motor_id]
        current_time = time.time()

        # หาก ROS feedback มีข้อมูล ให้ update ค่า feedback สำหรับ motor นี้
        if self.ros_interface.latest_joint_state is not None:
            js = self.ros_interface.latest_joint_state
            if motor_id is not None and f"motor_{motor_id}" in js.name:
                idx = js.name.index(f"motor_{motor_id}")
                # สมมติว่า:
                # - effort ใช้เป็น feedback current
                # - velocity ใช้เป็น feedback velocity (rad/s)
                # - position ใช้เป็น feedback position (อาจเป็น deg หรือ rad ขึ้นอยู่กับ node)
                fb_current = js.effort[idx] if idx < len(js.effort) else 0
                fb_velocity = js.velocity[idx] if idx < len(js.velocity) else 0
                fb_position = js.position[idx] if idx < len(js.position) else 0
                # บันทึก feedback ลง time series
                data['current_fb'].append(fb_current)
                data['velocity_fb'].append(fb_velocity)
                data['position_fb'].append(fb_position)
                data['time'].append(current_time)
            # หากไม่เจอมอเตอร์ที่เลือก ก็ไม่ update feedback

        # เพื่อไม่ให้ข้อมูลใน time series เติบโตไม่จำกัด เราสามารถตัดข้อมูลเก่าออก (เช่น เก็บแค่ 60 วินาที)
        time_window = 60  # วินาที
        min_time = current_time - time_window
        for key in data:
            # key 'time' ใช้ในการตัด
            pass
        # ตัดข้อมูลใน time seriesที่มีค่า time น้อยกว่า min_time
        indices = [i for i, t in enumerate(data['time']) if t >= min_time]
        for key in data:
            data[key] = [data[key][i] for i in indices]

        # เรียก plot ข้อมูล
        self.plot_canvas.plot_data(data['time'],
                                     data['current_cmd'], data['current_fb'],
                                     data['velocity_cmd'], data['velocity_fb'],
                                     data['position_cmd'], data['position_fb'])

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


###############################################################################
# Main Function
###############################################################################
def main():
    # เริ่มต้น ROS2
    rclpy.init()
    ros_interface = RosInterface()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_interface,), daemon=True)
    ros_thread.start()

    # สร้าง GUI
    app = QtWidgets.QApplication(sys.argv)
    config_path = "/home/kittinook/dynamixel_ws/src/dynamixel_ros2_multi_motor_control/config/motor_params.yaml"  # ปรับเป็น path ของไฟล์ config ของคุณ
    window = MainWindow(config_path, ros_interface)
    window.show()
    exit_code = app.exec_()

    ros_interface.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
