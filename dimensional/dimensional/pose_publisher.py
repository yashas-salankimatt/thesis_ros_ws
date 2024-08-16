import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, '/curr_target_pose', 10)
        self.timer = self.create_timer(2.0, self.publish_poses)

        # Define the hardcoded poses
        self.poses = []
        pose1 = Pose()
        pose1.position.x = 0.3
        pose1.position.y = -0.1
        pose1.position.z = 0.2
        pose1.orientation.x = 1.0
        pose1.orientation.y = 0.0
        pose1.orientation.z = 0.0
        pose1.orientation.w = 0.0
        self.poses.append(pose1)

        pose2 = Pose()
        pose2.position.x = 0.3
        pose2.position.y = 0.1
        pose2.position.z = 0.2
        pose2.orientation.x = 1.0
        pose2.orientation.y = 0.0
        pose2.orientation.z = 0.0
        pose2.orientation.w = 0.0
        self.poses.append(pose2)

        pose3 = Pose()
        pose3.position.x = 0.4
        pose3.position.y = -0.1
        pose3.position.z = 0.2
        pose3.orientation.x = 1.0
        pose3.orientation.y = 0.0
        pose3.orientation.z = 0.0
        pose3.orientation.w = 0.0
        self.poses.append(pose3)

        pose4 = Pose()
        pose4.position.x = 0.4
        pose4.position.y = 0.1
        pose4.position.z = 0.2
        pose4.orientation.x = 1.0
        pose4.orientation.y = 0.0
        pose4.orientation.z = 0.0
        pose4.orientation.w = 0.0
        self.poses.append(pose4)

        self.current_pose_index = 0

    def publish_poses(self):
        if self.current_pose_index < len(self.poses):
            current_pose = self.poses[self.current_pose_index]
            self.publisher_.publish(current_pose)
            self.get_logger().info(f'Published pose {self.current_pose_index + 1}')
            self.current_pose_index += 1
        else:
            self.get_logger().info('All poses published.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()

    rclpy.spin(pose_publisher)

    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
