#!/usr/bin/env python3
import sys
import signal
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSlider,
    QGridLayout,
)
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from control_msgs.msg import MultiDOFCommand
from geometry_msgs.msg import Transform  # 添加 Transform 的导入

class Joint:
    def __init__(self, name, min_rad, max_rad):
        self.name = name
        self.min_rad = min_rad
        self.max_rad = max_rad
        self.current_rad = (min_rad + max_rad) / 2.0  # 初始化为中间位置

class JointPositionControllerGUI(QMainWindow):
    def __init__(self, ros_node, joints):
        super().__init__()
        self.ros_node = ros_node
        self.joints = joints
        self.init_ui()
        self.init_ros()
        self.show()

    def init_ui(self):
        self.setWindowTitle("关节位置控制器")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()
        grid_layout = QGridLayout()

        self.sliders = []
        self.labels = []

        for idx, joint in enumerate(self.joints):
            # 关节名称标签
            name_label = QLabel(joint.name)
            grid_layout.addWidget(name_label, idx, 0)

            # 滑块
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)
            initial_slider_value = int(
                (joint.current_rad - joint.min_rad) / (joint.max_rad - joint.min_rad) * 1000
            )
            slider.setValue(initial_slider_value)
            slider.setTickInterval(100)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.valueChanged.connect(self.make_slider_callback(idx))
            grid_layout.addWidget(slider, idx, 1)
            self.sliders.append(slider)

            # 当前位置信息标签
            pos_label = QLabel(f"{joint.current_rad:.3f} rad")
            grid_layout.addWidget(pos_label, idx, 2)
            self.labels.append(pos_label)

        main_layout.addLayout(grid_layout)
        central_widget.setLayout(main_layout)

        # 定时器定期发布关节位置
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.publish_joint_positions)
        self.publish_timer.start(100)  # 以 10 Hz 的频率发布

    def make_slider_callback(self, joint_idx):
        def callback(value):
            joint = self.joints[joint_idx]
            # 将滑块值 (0-1000) 映射到关节位置 (min_rad 到 max_rad)
            joint.current_rad = joint.min_rad + (joint.max_rad - joint.min_rad) * (value / 1000.0)
            # 更新标签
            self.labels[joint_idx].setText(f"{joint.current_rad:.3f} rad")
        return callback

    def init_ros(self):
        # 创建发布者，发布到 /tita/pid_controller/reference
        self.publisher = self.ros_node.create_publisher(MultiDOFCommand, "/tita/pid_controller/reference", 10)

    def publish_joint_positions(self):
        msg = MultiDOFCommand()

        # 设置 degrees of freedom (DoF) 的名称
        msg.dof_names = [joint.name for joint in self.joints]

        # 设置值（位置）
        msg.values = [joint.current_rad for joint in self.joints]

        # 设置速度（如果需要，可以设为零）
        msg.values_dot = [0.0 for _ in self.joints]  # 假设速度为0

        # 发布消息
        self.publisher.publish(msg)


    def closeEvent(self, event):
        # 关闭窗口时停止定时器
        self.publish_timer.stop()
        event.accept()

class ROSNode(Node):
    def __init__(self):
        super().__init__('qt_joint_controller_gui')
        # 不需要订阅任何内容，只需发布

def main(args=None):
    # 初始化 ROS 2
    rclpy.init(args=args)

    # 定义关节及其限制（从 URDF 中获取）
    joints = [
        Joint("joint_left_leg_1", -0.785398, 0.785398),
        Joint("joint_left_leg_2", -1.919862, 3.490659),
        Joint("joint_left_leg_3", -2.670354, -0.698132),
        Joint("joint_right_leg_1", -0.785398, 0.785398),
        Joint("joint_right_leg_2", -1.919862, 3.490659),
        Joint("joint_right_leg_3", -2.670354, -0.698132),
    ]

    # 创建 ROS 节点
    ros_node = ROSNode()

    # 创建 Qt 应用程序
    app = QApplication(sys.argv)

    # 创建并显示 GUI
    gui = JointPositionControllerGUI(ros_node, joints)

    # 定义 SIGINT 的处理程序以优雅地关闭
    def signal_handler(sig, frame):
        print("收到 SIGINT. 正在关闭...")
        gui.close()  # 这将触发 closeEvent
        app.quit()

    # 注册信号处理程序
    signal.signal(signal.SIGINT, signal_handler)

    # 使用 QTimer 定期调用 rclpy.spin_once
    def ros_spin():
        rclpy.spin_once(ros_node, timeout_sec=0.1)

    ros_timer = QTimer()
    ros_timer.timeout.connect(ros_spin)
    ros_timer.start(2)  # 每 10 毫秒检查 ROS 消息

    # 执行 Qt 应用程序
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        print("收到 KeyboardInterrupt. 正在退出...")
        pass

    # 清理工作
    ros_timer.stop()
    gui.close()
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)

if __name__ == '__main__':
    main()
