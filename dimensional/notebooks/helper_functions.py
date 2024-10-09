import numpy as np
import matplotlib.pyplot as plt
import cv2
import sys
import time
import json
import os
import re
import pickle

def show_anns(anns):
    if len(anns) == 0:
        return
    sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)
    ax = plt.gca()
    ax.set_autoscale_on(False)

    img = np.ones((sorted_anns[0]['segmentation'].shape[0], sorted_anns[0]['segmentation'].shape[1], 4))
    img[:,:,3] = 0
    for ann in sorted_anns:
        m = ann['segmentation']
        color_mask = np.concatenate([np.random.random(3), [0.35]])
        img[m] = color_mask
    ax.imshow(img)


def show_anns_reg(anns):
    if len(anns) == 0:
        return
    # sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)
    ax = plt.gca()
    ax.set_autoscale_on(False)

    img = np.ones((anns[0]['segmentation'].shape[0], anns[0]['segmentation'].shape[1], 4))
    img[:,:,3] = 0
    for ann in anns:
        m = ann['segmentation']
        color_mask = np.concatenate([np.random.random(3), [0.35]])
        img[m] = color_mask
    ax.imshow(img)


# make a list of colors
def random_colors(N):
    np.random.seed(1)
    intensity = 0.6
    color = np.concatenate([np.random.random(3), np.array([intensity])], axis=0)
    colors = np.array([color])
    for i in range(N-1):
        color = np.concatenate([np.random.random(3), np.array([intensity])], axis=0)
        colors = np.concatenate([colors, [color]], axis=0)
    return colors

color_list = random_colors(100)

def show_mask(mask, ax, random_color=False, color=None):
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    elif color is not None:
        color = color
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)
    
def show_points(coords, labels, ax, marker_size=375):
    pos_points = coords[labels==1]
    neg_points = coords[labels==0]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color='green', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color='red', marker='*', s=marker_size, edgecolor='white', linewidth=1.25)   
    
def show_box(box, ax):
    x0, y0 = box[0], box[1]
    w, h = box[2] - box[0], box[3] - box[1]
    ax.add_patch(plt.Rectangle((x0, y0), w, h, edgecolor='green', facecolor=(0,0,0,0), lw=2))    


# getting the hu moments of the mask for shape dissimilarity
def hu_moments(mask):
    mask = mask.astype(np.uint8)
    moments = cv2.moments(mask)
    hu_moments = cv2.HuMoments(moments)
    return hu_moments

# calculating the shape dissimilarity
def shape_dissimilarity(hu_moments1, hu_moments2):
    diff = np.abs(hu_moments1 - hu_moments2)
    dissimilarity = np.mean(diff)
    return dissimilarity


# calculate the intersection over union (IOU) of two masks
def iou(mask1, mask2):
    intersection = np.logical_and(mask1, mask2)
    union = np.logical_or(mask1, mask2)
    return np.sum(intersection) / np.sum(union)


# function to get the bounding box given a mask, not using openCV
def get_box(segmentation_mask):
    rows = len(segmentation_mask)
    if rows == 0:
        return None
    cols = len(segmentation_mask[0])
    
    # Initialize bounding box coordinates
    min_row, min_col = rows, cols
    max_row, max_col = 0, 0
    
    # Iterate through the segmentation mask to find True values
    for row in range(rows):
        for col in range(cols):
            if segmentation_mask[row][col]:
                min_row = min(min_row, row)
                min_col = min(min_col, col)
                max_row = max(max_row, row)
                max_col = max(max_col, col)
    
    # If no True values found, return None
    if min_row == rows or min_col == cols:
        return None
    
    return [min_col, min_row, max_col, max_row]


def find_centroid(mask):
    mask = mask.astype(np.uint8)
    M = cv2.moments(mask)
    if M["m00"] == 0:
        return np.array([0, 0])
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return np.array([cX, cY])

def find_centroid_2(mask):
    # using np.argwhere and np.mean
    return np.mean(np.argwhere(mask), axis=0)


# Given an array of mask numbers, combine all the masks and return the combined mask
def combine_masks(mask_nums, masks):
    combined_mask = np.zeros(masks[0]['segmentation'].shape, dtype=bool)
    for i in mask_nums:
        combined_mask = np.logical_or(combined_mask, masks[i]['segmentation'])
    return combined_mask

# Given an np.array of shape (mask_num, width, height), return the combined mask
def combine_all_masks(masks):
    combined_mask = np.zeros(masks[0].shape, dtype=bool)
    for mask in masks:
        combined_mask = np.logical_or(combined_mask, mask)
    return combined_mask


# function to get annotations from the json file
def get_annotations(json_file):
    # Read the JSON data from the file
    data = None
    with open(json_file, 'r') as file:
        data = json.load(file)

    annotations = []
    # Iterate over each image in the JSON data
    for image_data in data:
        image_path = image_data['image']
        index = image_path.index("test-images")
        image_path = image_path[index:]
        image_path = image_path.replace("X", "/")
        original_width = image_data['label'][0]['original_width']
        original_height = image_data['label'][0]['original_height']
        
        # Create a dictionary to store the bounding boxes for each label
        bounding_boxes = {}
        
        # Iterate over each annotation for the current image
        for annotation in image_data['label']:
            label = annotation['rectanglelabels'][0]
            
            # Extract the bounding box coordinates
            x = annotation['x']
            y = annotation['y']
            width = annotation['width']
            height = annotation['height']
            
            # Scale the coordinates to the original image dimensions
            x_scaled = int(x * original_width / 100)
            y_scaled = int(y * original_height / 100)
            width_scaled = int(width * original_width / 100)
            height_scaled = int(height * original_height / 100)
            
            # Calculate the xyxy coordinates
            x1 = x_scaled
            y1 = y_scaled
            x2 = x_scaled + width_scaled
            y2 = y_scaled + height_scaled
            
            # Add the bounding box coordinates to the dictionary
            if label not in bounding_boxes:
                bounding_boxes[label] = []
            bounding_boxes[label].append([x1, y1, x2, y2])
        
        annotations.append((image_path, bounding_boxes))

    ann_np = np.array(annotations)
    return ann_np



# function that takes in a mask and returns the mask with the largest contiguous section still kept in the mask and nothing else
def get_largest_contiguous(mask):
    # get the largest contiguous section of the mask
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    largest_label = 1
    max_area = stats[1, cv2.CC_STAT_AREA]
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] > max_area:
            max_area = stats[i, cv2.CC_STAT_AREA]
            largest_label = i
    mask = np.array(labels == largest_label, dtype=bool)
    return mask


def sorted_alphanumeric(data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(data, key=alphanum_key)


# binarize mask based on number in mask and input number to binarize based on
# 0 is background, 1 is the first object, 2 is the second object, etc.
def binarize_mask(mask, num):
    # make a copy of the mask
    mask = mask.copy()
    mask[mask != num] = False
    mask[mask == num] = True
    return mask

def binarize_and_preprocess(mask, num):
    mask = binarize_mask(mask, num)
    # mask = get_largest_contiguous(mask)
    return mask


# import segmentations that are in a list of .npy files at the image directory/segmentations/
def import_segmentations(image_dir):
    seg_dir = image_dir + 'segmentations/'
    seg_files = [f for f in os.listdir(seg_dir) if f.endswith('.npy')]
    # seg_files.sort()
    seg_files = sorted_alphanumeric(seg_files)
    segmentations = np.array([])
    for seg_file in seg_files:
        seg = np.load(seg_dir + seg_file)
        # add seg to segmentations
        if len(segmentations) == 0:
            segmentations = np.array([seg])
        else:
            segmentations = np.concatenate([segmentations, [seg]], axis=0)
    return segmentations