import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# This is meant to test fake joint state publisher without the GUI

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'pillar_platform_joint_state', 10)
        timer_period = 0.1  # Publish at 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['pillar_platform_joint']
        joint_state.position = [-0.3]  # Joint positions set to 0
        joint_state.velocity = [0.0]  # Joint velocities set to 0

        self.publisher_.publish(joint_state)
        self.get_logger().info(f'Publishing joint states: position={joint_state.position[0]}, velocity={joint_state.velocity[0]}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
