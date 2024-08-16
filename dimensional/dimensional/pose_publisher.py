import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32
import time
import threading
import tf2_ros
import tf2_geometry_msgs

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Subscribers
        self.grasp_pose_sub = self.create_subscription(PoseStamped, '/grasp_pose', self.grasp_pose_callback, 10)
        self.target_pose_sub = self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        # Publishers
        self.publisher_ = self.create_publisher(Pose, '/curr_target_pose', 10)
        self.gripper_position_pub = self.create_publisher(Float32, '/gripper_position', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.grasp_pose_ = None
        self.target_pose_ = None

        self.lock_ = threading.Lock()
        self.called = False

    def grasp_pose_callback(self, msg):
        with self.lock_:
            self.grasp_pose_ = msg
            self.try_execute()

    def target_pose_callback(self, msg):
        with self.lock_:
            self.target_pose_ = msg
            self.try_execute()

    def try_execute(self):
        if self.grasp_pose_ is not None and self.target_pose_ is not None and not self.called:
            self.called = True
            self.get_logger().info('Executing grasp and target poses.')

            # Execute to grasp pose
            self.execute_pose(self.grasp_pose_)
            self.publish_gripper_position(0.1)

            # Execute to target pose
            self.execute_pose(self.target_pose_)
            self.publish_gripper_position(0.2)

            # Clear the poses after execution
            self.grasp_pose_ = None
            self.target_pose_ = None

    def execute_pose(self, pose_stamped):
        try:
            # Transform the pose to 'link_base' frame
            transform = self.tf_buffer.lookup_transform(
                'link_base',  # target frame
                pose_stamped.header.frame_id,  # source frame
                rclpy.time.Time())  # time
            
            # Extract Pose from PoseStamped
            posepub = pose_stamped.pose

            transformed_pose = tf2_geometry_msgs.do_transform_pose(posepub, transform)

            # Extract Pose from PoseStamped
            posepub = transformed_pose

            self.publisher_.publish(posepub)
            self.get_logger().info(f'Executing pose in link_base frame: {posepub}')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Could not transform pose: {e}')

    def publish_gripper_position(self, position):
        msg = Float32()
        msg.data = position
        self.gripper_position_pub.publish(msg)
        self.get_logger().info(f'Published gripper position: {position}')

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()

    rclpy.spin(pose_publisher)

    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
