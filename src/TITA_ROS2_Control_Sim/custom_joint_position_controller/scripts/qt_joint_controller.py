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
from std_msgs.msg import Float32MultiArray

class Joint:
    def __init__(self, name, min_rad, max_rad):
        self.name = name
        self.min_rad = min_rad
        self.max_rad = max_rad
        self.current_rad = (min_rad + max_rad) / 2.0  # Initialize to mid-position

class JointPositionControllerGUI(QMainWindow):
    def __init__(self, ros_node, joints):
        super().__init__()
        self.ros_node = ros_node
        self.joints = joints
        self.init_ui()
        self.init_ros()
        self.show()

    def init_ui(self):
        self.setWindowTitle("Joint Position Controller")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout()

        grid_layout = QGridLayout()

        self.sliders = []
        self.labels = []

        for idx, joint in enumerate(self.joints):
            # Joint name label
            name_label = QLabel(joint.name)
            grid_layout.addWidget(name_label, idx, 0)

            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)
            # Map initial joint position to slider value
            initial_slider_value = int(
                (joint.current_rad - joint.min_rad) / (joint.max_rad - joint.min_rad) * 1000
            )
            slider.setValue(initial_slider_value)
            slider.setTickInterval(100)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.valueChanged.connect(self.make_slider_callback(idx))
            grid_layout.addWidget(slider, idx, 1)
            self.sliders.append(slider)

            # Current position label
            pos_label = QLabel(f"{joint.current_rad:.3f} rad")
            grid_layout.addWidget(pos_label, idx, 2)
            self.labels.append(pos_label)

        main_layout.addLayout(grid_layout)
        central_widget.setLayout(main_layout)

        # Timer to periodically publish joint positions
        self.publish_timer = QTimer()
        self.publish_timer.timeout.connect(self.publish_joint_positions)
        self.publish_timer.start(100)  # Publish at 10 Hz

    def make_slider_callback(self, joint_idx):
        def callback(value):
            joint = self.joints[joint_idx]
            # Map slider value (0-1000) to joint position (min_rad to max_rad)
            joint.current_rad = joint.min_rad + (joint.max_rad - joint.min_rad) * (value / 1000.0)
            # Update label
            self.labels[joint_idx].setText(f"{joint.current_rad:.3f} rad")
        return callback

    def init_ros(self):
        # Create a publisher for /tita_pointfoot/actions
        self.publisher = self.ros_node.create_publisher(Float32MultiArray, "/tita_pointfoot/actions", 10)

    def publish_joint_positions(self):
        msg = Float32MultiArray()
        msg.data = [float(joint.current_rad) for joint in self.joints]
        self.publisher.publish(msg)

    def closeEvent(self, event):
        # Stop the timer when the window is closed
        self.publish_timer.stop()
        event.accept()

class ROSNode(Node):
    def __init__(self):
        super().__init__('qt_joint_controller_gui')
        # No subscriptions needed for GUI publishing

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Define joints with their limits (from URDF)
    joints = [
        Joint("joint_left_leg_1", -0.785398, 0.785398),
        Joint("joint_left_leg_2", -1.919862, 3.490659),
        Joint("joint_left_leg_3", -2.670354, -0.698132),
        Joint("joint_right_leg_1", -0.785398, 0.785398),
        Joint("joint_right_leg_2", -1.919862, 3.490659),
        Joint("joint_right_leg_3", -2.670354, -0.698132),
    ]

    # Create ROS node
    ros_node = ROSNode()

    # Create Qt application
    app = QApplication(sys.argv)

    # Create and show GUI
    gui = JointPositionControllerGUI(ros_node, joints)

    # Define a handler for SIGINT to gracefully shutdown
    def signal_handler(sig, frame):
        print("SIGINT received. Shutting down...")
        gui.close()  # This will trigger closeEvent
        app.quit()

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Use a QTimer to periodically call rclpy.spin_once
    def ros_spin():
        rclpy.spin_once(ros_node, timeout_sec=0.1)

    ros_timer = QTimer()
    ros_timer.timeout.connect(ros_spin)
    ros_timer.start(10)  # Check for ROS messages every 10 ms

    # Execute Qt application
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Exiting...")
        pass

    # Cleanup
    ros_timer.stop()
    gui.close()
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)

if __name__ == '__main__':
    main()
