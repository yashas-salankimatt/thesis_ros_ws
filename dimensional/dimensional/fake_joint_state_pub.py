import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [
            'pan_joint', 'tilt_joint', 
            'pillar_platform_joint', 
            'right_wheel_joint', 'left_wheel_joint'
        ]
        self.positions = [0.0] * len(self.joint_names)
        self.timer_period = 0.1  # Publish at 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def set_joint_position(self, index, value):
        # Update the joint position based on slider value
        self.positions[index] = value

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.positions
        joint_state.velocity = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)

        self.publisher_.publish(joint_state)
        self.get_logger().info(f'Publishing joint states: {self.positions}')

class JointControlGUI(QWidget):
    def __init__(self, joint_publisher):
        super().__init__()
        self.joint_publisher = joint_publisher
        self.joint_ranges = {
            'pan_joint': (0.0, 0.5),
            'tilt_joint': (0.0, 0.5),
            'pillar_platform_joint': (-1.0, 1.0),
            'right_wheel_joint': (-1.0, 1.0),
            'left_wheel_joint': (-1.0, 1.0)
        }
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Joint Control')
        layout = QVBoxLayout()

        self.sliders = []
        self.labels = []

        for i, joint_name in enumerate(self.joint_publisher.joint_names):
            min_val, max_val = self.joint_ranges.get(joint_name, (0.0, 0.5))
            label = QLabel(f'{joint_name}: {min_val}')
            layout.addWidget(label)
            self.labels.append(label)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)  # Scale slider to 1000 steps for precision
            slider.setValue(0)
            slider.setTickPosition(QSlider.TicksBelow)
            slider.setTickInterval(10)
            slider.valueChanged.connect(self.make_slider_callback(i, min_val, max_val))
            layout.addWidget(slider)
            self.sliders.append(slider)

        self.setLayout(layout)
        self.show()

    def make_slider_callback(self, index, min_val, max_val):
        def slider_callback(value):
            # Map slider value (0-1000) to joint range (min_val-max_val)
            position = min_val + (value / 1000.0) * (max_val - min_val)
            self.joint_publisher.set_joint_position(index, position)
            self.labels[index].setText(f'{self.joint_publisher.joint_names[index]}: {position:.2f}')
        return slider_callback

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()

    # Start the Qt application
    app = QApplication(sys.argv)
    gui = JointControlGUI(joint_state_publisher)

    # Spin ROS 2 node in a separate thread to keep publishing joint states
    rclpy_executor = rclpy.executors.MultiThreadedExecutor()
    rclpy_executor.add_node(joint_state_publisher)

    import threading
    ros_thread = threading.Thread(target=rclpy_executor.spin, daemon=True)
    ros_thread.start()

    # Run the Qt application
    app.exec_()

    # Cleanup
    joint_state_publisher.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
