import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, PointCloud2
import time
import threading
import tf2_ros
import tf2_geometry_msgs
from object_processor import ObjectProcessor

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.image_received = False
        self.point_cloud_received = False

        # Subscribe to image topic
        self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)
        self.create_subscription(PointCloud2, '/camera/point_cloud', self.point_cloud_callback, 10)

        # Publishers
        self.publisher_ = self.create_publisher(Pose, '/curr_target_pose', 10)
        self.gripper_position_pub = self.create_publisher(Float32, '/gripper_position', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Home pose
        self.home_pose_ = Pose()
        self.home_pose_.position.x = 0.0
        self.home_pose_.position.y = 0.0
        self.home_pose_.position.z = 0.65
        self.home_pose_.orientation.x = 0.952
        self.home_pose_.orientation.y = 0.0
        self.home_pose_.orientation.z = 0.307
        self.home_pose_.orientation.w = 0.0

        self.grasp_pose_ = None
        self.target_pose_ = None

        self.lock_ = threading.Lock()
        self.called = False

    def image_callback(self, image):
        if (not self.image_received):
            self.image = image
            self.image_received = True

    def point_cloud_callback(self, point_cloud):
        if (not self.point_cloud_received):
            self.point_cloud = point_cloud
            self.point_cloud_received = True

    def try_object_processing(self):
        if self.image_received and self.point_cloud_received:
            self.get_logger().info('Processing object.')
            self.grasp_object = ObjectProcessor(self.image, "a coffee mug.", self.point_cloud)

    def try_execute(self):
        if self.grasp_pose_ is not None and self.target_pose_ is not None and not self.called:
            self.called = True
            self.get_logger().info('Executing grasp and target poses.')

            # # Execute to home pose
            self.execute_pose(self.home_pose_, default_orientation=False)
            self.publish_gripper_position(0.0)
            time.sleep(10)

            # Execute to grasp pose
            self.execute_pose(self.grasp_pose_)
            time.sleep(10)
            self.publish_gripper_position(0.1)

            # Execute to target pose
            self.execute_pose(self.target_pose_)
            time.sleep(10)
            self.publish_gripper_position(0.0)

            # Clear the poses after execution
            self.grasp_pose_ = None
            self.target_pose_ = None

    def execute_pose(self, pose_stamped, default_orientation=True):
        try:
            # check if pose has header
            if not hasattr(pose_stamped, 'header'):
                posepub = pose_stamped
            else:
                if pose_stamped.header.frame_id != 'link_base':
                    self.get_logger().error(f'Pose not in link_base frame')
                posepub = pose_stamped.pose

                if default_orientation:
                    # Extract Pose from PoseStamped
                    posepub.orientation.x = 1.0
                    posepub.orientation.y = 0.0
                    posepub.orientation.z = 0.0
                    posepub.orientation.w = 0.0

                posepub.position.z += 0.2


            self.publisher_.publish(posepub)
            self.get_logger().info(f'\nExecuting pose in link_base frame: {posepub}')

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
