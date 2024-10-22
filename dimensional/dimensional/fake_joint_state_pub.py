import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt

class JointStatePublisher(Node):
    def __init__(self, default_positions):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = [
            'pan_joint', 'tilt_joint', 
            'pillar_platform_joint', 
            'right_wheel_joint', 'left_wheel_joint'
        ]
        self.positions = default_positions
        self.subscribers = []
        self.setup_subscribers()
        self.timer_period = 0.1  # Publish at 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def setup_subscribers(self):
        # Subscribe to each joint's "_state" topic
        for i, joint_name in enumerate(self.joint_names):
            topic_name = f'{joint_name}_state'
            subscriber = self.create_subscription(
                JointState, 
                topic_name, 
                self.make_joint_callback(i), 
                10
            )
            self.subscribers.append(subscriber)

    def make_joint_callback(self, index):
        # Update the joint position based on incoming JointState messages
        def joint_callback(msg):
            # Find the joint in the message
            if self.joint_names[index] in msg.name:
                joint_index = msg.name.index(self.joint_names[index])
                self.positions[index] = msg.position[joint_index]
                self.get_logger().info(f'Received {self.joint_names[index]} state: {msg.position[joint_index]}')
        return joint_callback

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
    def __init__(self, joint_publisher, default_positions):
        super().__init__()
        self.joint_publisher = joint_publisher
        self.joint_ranges = {
            'pan_joint': (-1.57, 1.57),
            'tilt_joint': (-1.57, 1.57),
            'pillar_platform_joint': (-0.85, 0.0),
            'right_wheel_joint': (-1.0, 1.0),
            'left_wheel_joint': (-1.0, 1.0)
        }
        self.default_positions = default_positions
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Joint Control')
        layout = QVBoxLayout()

        self.sliders = []
        self.labels = []

        for i, joint_name in enumerate(self.joint_publisher.joint_names):
            min_val, max_val = self.joint_ranges.get(joint_name, (0.0, 0.5))
            default_value = self.default_positions[i]
            label = QLabel(f'{joint_name}: {default_value}')
            layout.addWidget(label)
            self.labels.append(label)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(1000)  # Scale slider to 1000 steps for precision
            slider.setValue(int((default_value - min_val) / (max_val - min_val) * 1000))
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
    
    # Get parameter to determine if GUI should be launched
    node = rclpy.create_node('joint_state_node')
    launch_gui = node.declare_parameter('launch_gui', True).get_parameter_value().bool_value
    node.destroy_node()

    # Set default positions for each joint
    default_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # Modify this array to set different default values

    joint_state_publisher = JointStatePublisher(default_positions)

    # Start the Qt application if launch_gui is True
    if launch_gui:
        app = QApplication(sys.argv)
        gui = JointControlGUI(joint_state_publisher, default_positions)

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
    else:
        rclpy.spin(joint_state_publisher)
        joint_state_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
