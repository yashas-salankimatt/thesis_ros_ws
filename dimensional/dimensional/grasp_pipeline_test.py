#!/usr/bin/python3

import torch
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from PIL import Image as PILImage
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import tf2_geometry_msgs

def show_mask(mask, ax, random_color=False, borders = True):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask = mask.astype(np.uint8)
    mask_image =  mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    if borders:
        import cv2
        contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        # Try to smooth contours
        contours = [cv2.approxPolyDP(contour, epsilon=0.01, closed=True) for contour in contours]
        mask_image = cv2.drawContours(mask_image, contours, -1, (1, 1, 1, 0.5), thickness=2) 
    ax.imshow(mask_image)

def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels==1]
    neg_points = coords[labels==0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)   

def show_box(box, ax):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0, 0, 0, 0), lw=2))    

def show_masks(image, masks, scores, point_coords=None, box_coords=None, input_labels=None, borders=True):
    for i, (mask, score) in enumerate(zip(masks, scores)):
        plt.figure(figsize=(10, 10))
        plt.imshow(image)
        show_mask(mask, plt.gca(), borders=borders)
        if point_coords is not None:
            assert input_labels is not None
            show_points(point_coords, input_labels, plt.gca())
        if box_coords is not None:
            # boxes
            show_box(box_coords, plt.gca())
        if len(scores) > 1:
            plt.title(f"Mask {i+1}, Score: {score:.3f}", fontsize=18)
        plt.axis('off')
        plt.show()

def erode_masks(masks, kernel_size=[5,5], iterations=3):
    """Erode masks using a kernel of the given size and number of iterations."""
    kernel = np.ones((kernel_size[0], kernel_size[1]), np.uint8)  # You can adjust the kernel size to control the amount of shrinking
    if len(masks.shape) == 3:  # Multiple masks
        eroded_masks = []
        for mask in masks:
            eroded_mask = cv2.erode(mask.astype(np.uint8), kernel, iterations=iterations)
            eroded_masks.append(eroded_mask)
        eroded_masks = np.stack(eroded_masks, axis=0)
    elif len(masks.shape) == 2:  # Single mask
        eroded_masks = cv2.erode(masks.astype(np.uint8), kernel, iterations=iterations)
    else:
        raise ValueError("Unexpected mask shape: {}".format(masks.shape))
    return eroded_masks

def get_mask_dimensions(mask):
    """
    Finds the min and max x, y coordinates of the non-zero values in the mask
    and calculates the width and height of the bounding box.
    
    Parameters:
    - mask (numpy.ndarray): A 2D array (e.g., 800x1200) representing the binary mask.
    
    Returns:
    - min_x, min_y, max_x, max_y: The coordinates of the bounding box around the non-zero mask area.
    - width (int): The width of the bounding box.
    - height (int): The height of the bounding box.
    """
    # Find the coordinates of all non-zero points in the mask
    y_indices, x_indices = np.nonzero(mask)
    
    # Calculate the bounding box around the non-zero points
    min_x, max_x = np.min(x_indices), np.max(x_indices)
    min_y, max_y = np.min(y_indices), np.max(y_indices)
    
    # Calculate the width and height of the bounding box
    width = max_x - min_x + 1  # +1 to include the boundary
    height = max_y - min_y + 1  # +1 to include the boundary
    
    return min_x, min_y, max_x, max_y, width, height

class GraspPipelineTest(Node):
    def __init__(self):
        super().__init__('grasp_pipeline_test')
        self.get_logger().info('Grasp pipeline test node started')

        # Load the model
        model_id = "IDEA-Research/grounding-dino-tiny"
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(device=self.device)

        self.called = False
        self.point_cloud_saved = False
        self.object_pub = False
        self.target_pub = False
        self.grasp_pub = False
        self.target_pose_pub = False

        # Subscribe to the RGB image topic
        # self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/color/image', self.image_callback, 10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Subscribe to the point cloud topic
        # self.create_subscription(PointCloud2, '/camera/point_cloud', self.point_cloud_callback, 10)
        self.create_subscription(PointCloud2, '/stereo/points', self.point_cloud_callback, qos_profile)

        # Add new publishers and subscribers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/object_pointcloud', 10)
        self.target_pointcloud_pub = self.create_publisher(PointCloud2, '/target_pointcloud', 10)
        self.grasp_pose_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        self.target_pose_publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        
        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def point_cloud_callback(self, point_cloud):
        print("Received point cloud.")
        self.point_cloud = point_cloud
        self.point_cloud_saved = True
        transform = self.tf_buffer.lookup_transform('world', 'link6', rclpy.time.Time())
        if self.object_pub:
            # print(self.point_cloud.data.shape)
            # print(self.object_cloud.data.shape)
            # self.point_cloud.data = self.object_cloud.data
            self.object_cloud.header.stamp = transform.header.stamp
            self.pointcloud_pub.publish(self.object_cloud)
            print("Published object point cloud")
        if self.target_pub:
            self.target_cloud.header.stamp = transform.header.stamp
            self.target_pointcloud_pub.publish(self.target_cloud)
            print("Published target point cloud")
        if self.grasp_pub:
            self.grasp_pose.header.stamp = transform.header.stamp
            self.grasp_pose_pub.publish(self.grasp_pose)
            print("Published grasp pose")
        if self.target_pose_pub:
            self.target_pose.header.stamp = transform.header.stamp
            self.target_pose_publisher.publish(self.target_pose)
            print("Published target pose")

    def image_callback(self, image):
        if (self.called or not self.point_cloud_saved):
            return

        self.called = True
        # Convert the ROS Image message to a PIL Image
        img = PILImage.frombytes("RGB", (image.width, image.height), bytes(image.data), "raw")
        # visualize the image
        # img.show()
        text = "can."

        # Perform object detection
        inputs = self.processor(images=img, text=text, return_tensors="pt").to(device=self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            box_threshold=0.4,
            text_threshold=0.3,
            target_sizes=[img.size[::-1]]
        )

        print(results)

        # visualize bounding boxes on top of image
        # img = cv2.cvtColor(cv2.imread("image.jpg"), cv2.COLOR_BGR2RGB)
        # img = img.convert("RGB")
        # convert img to numpy array
        # img_disp = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        for box in results[0]["boxes"]:
            print(box)
            x_min, y_min, x_max, y_max = box
            x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
            # cv2.rectangle(img_disp, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        # cv2.imshow("image", img_disp)
        # cv2.waitKey(0) # wait for any key press

        # use SAM2 to get segmentation mask of object
        print("Segmenting object...")
        # sam2_checkpoint = "/home/yashas/Documents/thesis/segment-anything-2/checkpoints/sam2_hiera_tiny.pt"
        sam2_checkpoint = "/home/ros/deps/segment-anything-2/checkpoints/sam2_hiera_tiny.pt"
        model_cfg = "sam2_hiera_t.yaml"
        sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=self.device)
        predictor = SAM2ImagePredictor(sam2_model)
        predictor.set_image(img)
        input_box = np.array([x_min, y_min, x_max, y_max])
        masks, scores, _ = predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_box[None, :],
            multimask_output=False,
        )
        min_x, min_y, max_x, max_y, width, height = get_mask_dimensions(masks[0])
        print(f"Mask dimensions: {width}x{height}")
        masks = erode_masks(masks, kernel_size=[int(width/10),int(height/10)], iterations=1)


        # Get the intersection between point cloud and object mask
        object_points = self.get_object_points(masks[0], self.point_cloud)
        
        # Publish the object point cloud
        self.publish_object_pointcloud(object_points)

        target_text = "cardboard box."
        target_inputs = self.processor(images=img, text=target_text, return_tensors="pt").to(device=self.device)
        with torch.no_grad():
            target_outputs = self.model(**target_inputs)

        target_results = self.processor.post_process_grounded_object_detection(
            target_outputs,
            target_inputs.input_ids,
            box_threshold=0.4,
            text_threshold=0.3,
            target_sizes=[img.size[::-1]]
        )

        print(target_results)

        # visualize bounding boxes on top of image
        # img = cv2.cvtColor(cv2.imread("image.jpg"), cv2.COLOR_BGR2RGB)
        # img = img.convert("RGB")
        # convert img to numpy array
        # img_disp = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        for box in target_results[0]["boxes"]:
            print(box)
            x_min, y_min, x_max, y_max = box
            x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
            # cv2.rectangle(img_disp, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        print("Segmenting target...")
        predictor.set_image(img)
        input_box = np.array([x_min, y_min, x_max, y_max])
        masks, scores, _ = predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_box[None, :],
            multimask_output=False,
        )
        min_x, min_y, max_x, max_y, width, height = get_mask_dimensions(masks[0])
        print(f"Mask dimensions: {width}x{height}")
        masks = erode_masks(masks, kernel_size=[int(width/10),int(height/10)], iterations=1)

        # Get the intersection between point cloud and object mask
        print("Getting target points...")
        target_points = self.get_object_points(masks[0], self.point_cloud)

        # Publish the object point cloud
        self.publish_target_pointcloud(target_points)

        # Calculate grasp pose
        print("Calculating grasp pose...")
        self.grasp_pose = self.calculate_grasp_pose(object_points)
        # self.publish_grasp_pose(self.grasp_pose)
        self.grasp_pub = True
        print(self.grasp_pose)

        # Calculate target pose
        print("Calculating target pose...")
        self.target_pose = self.calculate_grasp_pose(target_points)
        # self.publish_target_pose(self.target_pose)
        self.target_pose_pub = True
        print(self.target_pose)
        
        # Move robot to grasp pose
        # self.move_to_grasp_pose(grasp_pose)
        return

    def get_object_points(self, mask, point_cloud):
        print("Getting object points...")
    
        # Convert PointCloud2 to numpy array
        pc = point_cloud2.read_points(point_cloud, skip_nans=True)
        points = np.array(list(pc))

        points_in_object = []

        # Intrinsic matrix for the camera (3x3) matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        # fx, fy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1]
        # cx, cy = intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]
        fx, fy = 1024.122, 1024.122
        cx, cy = 644.33, 367.452

        for point in points:
            # Project 3D point to 2D pixel coordinates
            u = int(fx * point['x'] / point['z'] + cx)
            v = int(fy * point['y'] / point['z'] + cy)
            
            # Check if the pixel is within mask boundaries
            if 0 <= u < mask.shape[1] and 0 <= v < mask.shape[0]:
                # Check if the pixel corresponds to a '1' in the segmentation mask
                if mask[v, u] == 1:
                    points_in_object.append(point)
        
        points_in_object = np.array(points_in_object)
        
        print(f"Original points: {len(points)}, Filtered points: {len(points_in_object)}")
        
        self.object_points = points_in_object
        
        return points_in_object

    def publish_object_pointcloud(self, object_points):
        # self.object_cloud = point_cloud2.create_cloud(self.point_cloud.header, self.point_cloud.fields, object_points)
        # Ensure object_points is in the format [(x, y, z), (x, y, z), ...]
        xyz_points = [(point['x'], point['y'], point['z']) for point in object_points]

        # Create the PointCloud2 message
        self.object_cloud = point_cloud2.create_cloud_xyz32(self.point_cloud.header, xyz_points)

        self.object_pub = True

    def publish_target_pointcloud(self, target_points):
        # self.target_cloud = point_cloud2.create_cloud(self.point_cloud.header, self.point_cloud.fields, target_points)
        # Ensure target_points is in the format [(x, y, z), (x, y, z), ...]
        xyz_points = [(point['x'], point['y'], point['z']) for point in target_points]

        # Create the PointCloud2 message
        self.target_cloud = point_cloud2.create_cloud_xyz32(self.point_cloud.header, xyz_points)

        self.target_pub = True

    def calculate_grasp_pose(self, object_points):
        # Simple grasp pose calculation - can be improved based on object geometry
        centroid = [0, 0, 0]
        min_dim = [float('inf'), float('inf'), float('inf')]
        max_dim = [float('-inf'), float('-inf'), float('-inf')]
        count = 0
        for i in range(len(object_points)):
            if object_points[i]['x'] == 0.0 and object_points[i]['y'] == 0.0 and object_points[i]['z'] == 0.0:
                continue
            if object_points[i]['x'] < min_dim[0]:
                min_dim[0] = object_points[i]['x']
            if object_points[i]['y'] < min_dim[1]:
                min_dim[1] = object_points[i]['y']
            if object_points[i]['z'] < min_dim[2]:
                min_dim[2] = object_points[i]['z']
            if object_points[i]['x'] > max_dim[0]:
                max_dim[0] = object_points[i]['x']
            if object_points[i]['y'] > max_dim[1]:
                max_dim[1] = object_points[i]['y']
            if object_points[i]['z'] > max_dim[2]:
                max_dim[2] = object_points[i]['z']

            centroid[0] += object_points[i]['x']
            centroid[1] += object_points[i]['y']
            centroid[2] += object_points[i]['z']
            count += 1
        # centroid = np.array(centroid) / len(object_points)
        centroid = np.array(centroid) / count

        self.get_logger().info(f"Object centroid: {centroid}")
        self.get_logger().info(f"Object min dim: {min_dim}")
        self.get_logger().info(f"Object max dim: {max_dim}")
        
        # Create a pose
        pose = PoseStamped()
        pose.header.frame_id = self.point_cloud.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = centroid[0]
        pose.pose.position.y = centroid[1]
        pose.pose.position.z = centroid[2]
        # pose.pose.position.x = (min_dim[0] + max_dim[0]) / 2
        # pose.pose.position.y = (min_dim[1] + max_dim[1]) / 2
        # pose.pose.position.z = (min_dim[2] + max_dim[2]) / 2
        
        # Set orientation (this is a simple orientation, you might want to improve it)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        # Get the transform from the object frame to the 'link_base' frame
        transform = self.tf_buffer.lookup_transform(
            'link_base',
            self.point_cloud.header.frame_id,
            rclpy.time.Time())

        # Transform the pose to the 'link_base' frame
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose.pose, transform)

        # Extract Pose from PoseStamped
        posepubstamped = PoseStamped()
        posepubstamped.header.frame_id = 'link_base'
        posepubstamped.header.stamp = self.get_clock().now().to_msg()
        posepubstamped.pose = transformed_pose

        return posepubstamped

def main(args=None):
    print("Starting grasp pipeline test node...")
    rclpy.init(args=args)
    node = GraspPipelineTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
