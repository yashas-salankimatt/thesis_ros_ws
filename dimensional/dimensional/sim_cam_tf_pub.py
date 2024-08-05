#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Define the static transform from link6 to the camera
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'link6'
        static_transform.child_frame_id = 'camera_frame'

        # Camera is 10cm above link6 in the x axis
        static_transform.transform.translation.x = 0.1  # 10 cm
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0

        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info('Static transform from link6 to camera_frame published')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
