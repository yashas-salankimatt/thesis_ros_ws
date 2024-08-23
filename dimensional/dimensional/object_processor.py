#!/usr/bin/python3

import torch
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from PIL import Image as PILImage
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
import cv2
import numpy as np
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

class ObjectProcessor(object):
    def __init__(self, image, text: str, point_cloud: PointCloud2):
        if isinstance(image, PILImage):
            self.image: PILImage = image
        elif isinstance(image, Image):
            self.image: PILImage = PILImage.frombytes("RGB", (image.width, image.height), bytes(image.data), "raw")
        else:
            print("ERROR: Image must be a PIL Image or ROS Image")
            return

        self.object_text: str = text
        self.processed = False
        self.point_cloud: PointCloud2 = point_cloud

        if point_cloud.header.frame_id != 'world':
            print("ERROR: Point cloud frame is not world")

        # Check for CUDA
        if torch.cuda.is_available():
            print("CUDA is available")
            self.device: str = "cuda"
        else:
            print("CUDA is not available")
            self.device: str = "cpu"

        self.segmented_points = []
        try:
            self.ground_text()
            self.segment_image()
            self.segment_point_cloud()
            self.get_object_border_points()
            self.processed = True
        except Exception as e:
            print("Failed to process object with error: {}".format(e))
            return

    def ground_text(self):
        """
        Ground the text in the image.
        Uses GroundingDino to get the bounding box of the text in self.text

        Returns:
        - [x_min, y_min, x_max, y_max]: The bounding box of the text in the image
        """
        print("Grounding text: {}".format(self.object_text))
        model_id = "IDEA-Research/grounding-dino-tiny"
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(device=self.device)

        inputs = self.processor(images=self.image, text=self.object_text, return_tensors="pt").to(device=self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            box_threshold=0.4,
            text_threshold=0.3,
            target_sizes=[self.image.size[::-1]]
        )

        for box in results[0]["boxes"]:
            x_min, y_min, x_max, y_max = box
            x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
            self.bounding_box_points = [x_min, y_min, x_max, y_max]
            return [x_min, y_min, x_max, y_max]
        
        self.bounding_box_points = []

        print("ERROR: No {} detected in image".format(self.object_text))
        return None

    def segment_image(self, erode=True, percentage=0.1):
        """
        Segment the image using SAM2.
        Uses SAM2 to segment the image into objects and background.
        
        Args:
        - erode (bool): Whether to erode the masks before segmenting the image.
        - percentage (float): The percentage of pixels to erode the masks by

        Returns:
        - mask (numpy.ndarray): The mask for the object in the image.
        """
        print("Segmenting image for object: {}".format(self.object_text))
        sam2_checkpoint = "/home/ros/deps/segment-anything-2/checkpoints/sam2_hiera_tiny.pt"
        model_cfg = "sam2_hiera_t.yaml"
        sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=self.device)
        predictor = SAM2ImagePredictor(sam2_model)
        predictor.set_image(self.image)
        input_box = np.array([self.bounding_box_points[0], self.bounding_box_points[1], self.bounding_box_points[2], self.bounding_box_points[3]])
        masks, scores, _ = predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_box[None, :],
            multimask_output=False,
        )
        if erode:
            y_indices, x_indices = np.nonzero(masks[0])
            min_x, max_x = np.min(x_indices), np.max(x_indices)
            min_y, max_y = np.min(y_indices), np.max(y_indices)
            width = max_x - min_x + 1  # +1 to include the boundary
            height = max_y - min_y + 1  # +1 to include the boundary

            masks = self.erode_masks(masks, kernel_size=[int(width*percentage),int(height*percentage)], iterations=1)

        mask = masks[0]
        self.mask = mask
        return mask
    
    def segment_point_cloud(self):
        """
        Segment the point cloud using the mask in self.mask and the point cloud in self.point_cloud.

        Returns:
        - segmented_points (ndarray): The ndarray representation of the segmented point cloud.
        """
        print("Segmenting point cloud...")
        pc = point_cloud2.read_points(self.point_cloud, skip_nans=True)
        points = np.array(list(pc))
        points_in_object = np.copy(points)

        count = 0
        for i in range(self.mask.shape[1]):
            for j in range(self.mask.shape[0]):
                if self.mask[j][i] == 0:
                    points_in_object[j * self.mask.shape[1] + i]['x'] = 0.0
                    points_in_object[j * self.mask.shape[1] + i]['y'] = 0.0
                    points_in_object[j * self.mask.shape[1] + i]['z'] = 0.0
                    count += 1

        # return points_in_object
        self.segmented_points = points_in_object
        return points_in_object
    
    def get_object_border_points(self):
        """
        Get the 3D bounding box of the object in the point cloud.
        Uses the centroid and min/max coordinates of the object in the point cloud.
        """
        object_border_points = []
        centroid = [0, 0, 0]
        min_dim = [float('inf'), float('inf'), float('inf')]
        max_dim = [float('-inf'), float('-inf'), float('-inf')]
        count = 0
        for i in range(len(self.segmented_points)):
            if self.segmented_points[i]['x'] < min_dim[0]:
                min_dim[0] = self.segmented_points[i]['x']
            if self.segmented_points[i]['y'] < min_dim[1]:
                min_dim[1] = self.segmented_points[i]['y']
            if self.segmented_points[i]['z'] < min_dim[2]:
                min_dim[2] = self.segmented_points[i]['z']
            if self.segmented_points[i]['x'] > max_dim[0]:
                max_dim[0] = self.segmented_points[i]['x']
            if self.segmented_points[i]['y'] > max_dim[1]:
                max_dim[1] = self.segmented_points[i]['y']
            if self.segmented_points[i]['z'] > max_dim[2]:
                max_dim[2] = self.segmented_points[i]['z']

            centroid[0] += self.segmented_points[i]['x']
            centroid[1] += self.segmented_points[i]['y']
            centroid[2] += self.segmented_points[i]['z']
            count += 1

        centroid = np.array(centroid) / count

        print("Object centroid: {}".format(centroid))
        print("Object min dim: {}".format(min_dim))
        print("Object max dim: {}".format(max_dim))

        self.centroid = centroid
        self.min_coords = min_dim
        self.max_coords = max_dim

    
    def erode_masks(self, masks, kernel_size=[5,5], iterations=1):
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

    def get_mask_dimensions(self, mask):
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