#!/usr/bin/python3

import requests
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

class GraspPipelineTest(Node):
    def __init__(self):
        super().__init__('grasp_pipeline_test')
        self.get_logger().info('Grasp pipeline test node started')

        # Load the model
        model_id = "IDEA-Research/grounding-dino-tiny"
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id)

        self.called = False

        # Subscribe to the RGB image topic
        self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)

    def image_callback(self, image):
        if (self.called):
            return

        self.called = True
        # Convert the ROS Image message to a PIL Image
        img = PILImage.frombytes("RGB", (image.width, image.height), bytes(image.data), "raw")
        # visualize the image
        img.show()
        text = "a coffee mug."

        # Perform object detection
        inputs = self.processor(images=img, text=text, return_tensors="pt")
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
        img_disp = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        for box in results[0]["boxes"]:
            print(box)
            x_min, y_min, x_max, y_max = box
            x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
            cv2.rectangle(img_disp, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        cv2.imshow("image", img_disp)
        # cv2.waitKey(0) # wait for any key press

        # use SAM2 to get segmentation mask of object
        sam2_checkpoint = "/home/yashas/Documents/thesis/segment-anything-2/checkpoints/sam2_hiera_tiny.pt"
        model_cfg = "sam2_hiera_t.yaml"
        sam2_model = build_sam2(model_cfg, sam2_checkpoint, device="cpu")
        predictor = SAM2ImagePredictor(sam2_model)
        predictor.set_image(img)
        input_box = np.array([x_min, y_min, x_max, y_max])
        masks, scores, _ = predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_box[None, :],
            multimask_output=False,
        )
        show_masks(img, masks, scores, box_coords=input_box)

        return



def main(args=None):
    rclpy.init(args=args)
    node = GraspPipelineTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()
