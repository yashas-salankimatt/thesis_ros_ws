#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from scipy.spatial.transform import Rotation as R
import numpy as np


class TfToTwistPublisher(Node):
    def __init__(self):
        super().__init__('tf_to_twist_publisher')

        # Declare parameters with default values
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'alfred_base_link')
        self.declare_parameter('twist_topic', '/alfred_base_link_tf_twist')

        # Get parameter values
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value

        # Initialize the TF buffer and listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
        # Publisher to publish Twist messages
        self.twist_pub = self.create_publisher(Twist, self.twist_topic, 10)
        
        # Timer to periodically check for the transformation
        self.timer = self.create_timer(0.1, self.publish_twist_from_tf)
        self.get_logger().info("TF to Twist Publisher Node Started")
        
    def publish_twist_from_tf(self):
        try:
            # Look up transformation using specified frames
            trans = self.buffer.lookup_transform(self.parent_frame, self.child_frame, rclpy.time.Time())
            
            # Extract translation
            translation = trans.transform.translation
            # Extract rotation (quaternion)
            rotation = trans.transform.rotation
            # Convert quaternion to Euler angles using scipy
            rotation = trans.transform.rotation
            r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
            roll, pitch, yaw = r.as_euler('xyz', degrees=False)

            # Convert to angles
            roll = roll * 180 / np.pi
            pitch = pitch * 180 / np.pi
            yaw = yaw * 180 / np.pi
            
            # Print the transformation data
            # self.get_logger().info(
            #     f"Translation: x={translation.x}, y={translation.y}, z={translation.z}\n"
            #     f"Rotation: roll={roll}, pitch={pitch}, yaw={yaw}"
            # )

            
            # Populate the Twist message
            twist_msg = Twist()
            twist_msg.linear.x = translation.x
            twist_msg.linear.y = translation.y
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = roll
            twist_msg.angular.y = pitch
            twist_msg.angular.z = yaw-90
            # twist_msg.angular.x = 0.0
            # twist_msg.angular.y = 0.0
            # twist_msg.angular.z = 0.0

            # self.get_logger().info(f"Publishing Twist: {twist_msg}")

            
            # Publish the Twist message
            self.twist_pub.publish(twist_msg)
        
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TfToTwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
