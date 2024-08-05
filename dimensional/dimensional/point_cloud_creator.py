#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class PointCloudCreator(Node):
    def __init__(self):
        super().__init__('point_cloud_creator')
        self.bridge = CvBridge()

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers for RGB and Depth images
        self.rgb_sub = Subscriber(self, Image, '/camera/rgb/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')

        # Synchronizer to synchronize the RGB and Depth topics
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        # Publisher for the point cloud
        self.point_cloud_pub = self.create_publisher(PointCloud2, '/camera/point_cloud', 10)

    def callback(self, rgb_data, depth_data):
        try:
            # Convert ROS Image messages to OpenCV images
            cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_data, desired_encoding="bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding="32FC1")

            # Get the depth array
            depth_array = np.array(cv_depth_image, dtype=np.float32)

            # Log the min and max depth values
            min_depth = np.min(depth_array)
            max_depth = np.max(depth_array)
            self.get_logger().info(f"Min depth: {min_depth}")
            self.get_logger().info(f"Max depth: {max_depth}")

            # Ensure depth values are not scaled
            valid_depth_mask = depth_array > 0
            depth_array[~valid_depth_mask] = 0.0  # Replace invalid depths with zeros

            # Create Open3D RGBD image
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(cv_rgb_image),
                o3d.geometry.Image(depth_array),
                convert_rgb_to_intensity=False,
                depth_trunc=max_depth + 1.0,  # Ensure all valid depths are included
                depth_scale=1.0  # Prevent unintended scaling
            )

            # Camera intrinsic parameters (These need to be adjusted to your camera's parameters)
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            )
            intrinsic.set_intrinsics(
                width=rgb_data.width, 
                height=rgb_data.height, 
                fx=835.555,  # Focal length in x axis
                fy=835.555,  # Focal length in y axis
                cx=rgb_data.width / 2,  # Optical center in x axis
                cy=rgb_data.height / 2  # Optical center in y axis
            )

            # Generate the point cloud
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image,
                intrinsic
            )

            # Convert Open3D point cloud to numpy array
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors) * 255  # Convert from [0, 1] to [0, 255]
            colors = colors.astype(np.uint8)

            # Transform the points to align with ROS coordinate system
            # From Open3D camera coordinates to ROS coordinates:
            # x_ros = z_open3d, y_ros = -x_open3d, z_ros = -y_open3d
            transformed_points = np.zeros_like(points)
            # transformed_points[:, 0] = points[:, 2]  # x_ros = z_open3d
            # transformed_points[:, 1] = -points[:, 0]  # y_ros = -x_open3d
            # transformed_points[:, 2] = -points[:, 1]  # z_ros = -y_open3d
            transformed_points[:, 0] = -points[:, 1]  # x_ros = z_open3d
            transformed_points[:, 1] = points[:, 0]  # y_ros = -x_open3d
            transformed_points[:, 2] = points[:, 2]  # z_ros = -y_open3d

            # Check if points and colors are being generated
            if transformed_points.shape[0] == 0:
                self.get_logger().warn("No points generated in the point cloud.")
                return

            # Create structured array for point cloud
            structured_array = np.zeros(transformed_points.shape[0], dtype=[
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('r', np.uint8), ('g', np.uint8), ('b', np.uint8)
            ])
            structured_array['x'] = transformed_points[:, 0]
            structured_array['y'] = transformed_points[:, 1]
            structured_array['z'] = transformed_points[:, 2]
            structured_array['r'] = colors[:, 0]
            structured_array['g'] = colors[:, 1]
            structured_array['b'] = colors[:, 2]

            # Log the min and max depth values of the point cloud
            min_point_depth = np.min(structured_array['z'])
            max_point_depth = np.max(structured_array['z'])
            self.get_logger().info(f"Min point depth: {min_point_depth}")
            self.get_logger().info(f"Max point depth: {max_point_depth}")

            # Define the fields for PointCloud2
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
                PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
                PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
            ]

            # Create the PointCloud2 message
            # header = rgb_data.header
            # header.frame_id = 'link6'  # Set the frame to 'world'
            # point_cloud_msg = point_cloud2.create_cloud(header, fields, structured_array)
            # Adjust timestamp
            try:
                # Lookup the latest transform from world to link6
                transform = self.tf_buffer.lookup_transform('world', 'camera_frame', rclpy.time.Time())

                # Use the transform's timestamp for the point cloud message
                header = rgb_data.header
                header.stamp = transform.header.stamp
                header.frame_id = 'camera_frame'  # Set the frame to 'link6'
            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().error(f"Could not transform: {ex}")
                return

            # Create the PointCloud2 message
            point_cloud_msg = point_cloud2.create_cloud(header, fields, structured_array)


            # Publish the point cloud
            self.point_cloud_pub.publish(point_cloud_msg)

            self.get_logger().info('Published point cloud')

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    point_cloud_creator = PointCloudCreator()
    try:
        rclpy.spin(point_cloud_creator)
    except KeyboardInterrupt:
        point_cloud_creator.get_logger().info("Shutting down")
    finally:
        point_cloud_creator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
