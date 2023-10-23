#!/usr/bin/env python

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

import cv2
import glob
import numpy as np

import tensorflow as tf
from tensorflow.keras.metrics import MeanIoU, IoU

from PIL import Image
from IPython.display import clear_output

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

plt.rcParams['figure.figsize'] = (12, 12) 

# Define a class to evaluate mean IoU (Intersection over Union)
class Evaluate_miou:
    
    def __init__(self):
        
        # Define a mapping from RGB colors to class IDs for segmentation
        self.rgb_to_class_id = {
                        (152, 251, 152): 0, # terrain/ground
                        (128,  64, 128): 1, # road
                        (244,  35, 232): 2, # sidewalk/parking
                        ( 70,  70,  70): 3, # building/wall/fence/pole/obstacle/...
                        (107, 142,  35): 4, # vegetation
                        (255,   0,   0): 5, # person
                        (220,  20,  60): 6, # rider/motorcycle/bicycle
                        (  0,   0, 142): 7, # car/truck/bus/van
                        (  0, 145, 198): 8, # sky
                        }
        
        self.num_classes = len(self.rgb_to_class_id)
        
        self.class_names = [
            "Class 0 - Terrain/Ground",
            "Class 1 - Road",
            "Class 2 - Sidewalk/Parking",
            "Class 3 - Building/Obstacle",
            "Class 4 - Vegetation",
            "Class 5 - Person",
            "Class 6 - Rider/Motorcycle",
            "Class 7 - Car/Truck/Bus/Van",
            "Class 8 - Sky"
        ]
        
    def segmentation_map_to_rgb_encoding(self, segmentation_map):
        # Convert segmentation map to RGB encoding
        rgb_encoding = np.zeros([segmentation_map.shape[0], segmentation_map.shape[1], 3], dtype=np.uint8)
        for color, class_id in self.rgb_to_class_id.items():
            rgb_encoding[segmentation_map==class_id] = color
        return rgb_encoding
    
    def parse_sample(self, image_rgb, label_rgb):
        # Resize the images to desired size 
        image_rgb = tf.image.resize(image_rgb, [512, 1024], method=tf.image.ResizeMethod.NEAREST_NEIGHBOR)
        label_rgb = tf.image.resize(label_rgb, [512, 1024], method=tf.image.ResizeMethod.NEAREST_NEIGHBOR)
        image_rgb = tf.cast(image_rgb, tf.uint8)
        image_segmentation_map = self.convert_rgb_encoding_to_segmentation_map(image_rgb)
        label_segmentation_map = self.convert_rgb_encoding_to_segmentation_map(label_rgb)
        return image_segmentation_map, label_segmentation_map
    
    def convert_rgb_encoding_to_segmentation_map(self, image):
        # Convert the RGB encoded image to a segmentation map
        segmentation_map = tf.zeros([image.shape[0], image.shape[1]], dtype=tf.uint8)
        for color, class_id in self.rgb_to_class_id.items():    
            segmentation_map = tf.where(condition=tf.reduce_all(tf.equal(image, color), axis=-1),
                x=tf.cast(class_id, tf.uint8),
                y=segmentation_map)
        segmentation_map = tf.expand_dims(segmentation_map, -1)
        return segmentation_map
    
    def get_image_from_file(self, ipm_file):
        img = np.load(ipm_file)['data']
        img_corrected = img
        fractional, integral = np.modf(img[...,0])
        img_corrected[...,0][fractional==0] -= 1
        img_class = img_corrected.astype(int)
        img_class_0 = img_class[:,:,0]
        img_rgb = self.segmentation_map_to_rgb_encoding(img_class_0)
        ipm_image = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
        return ipm_image
    
    def compute_class_ious_miou(self, prediction_path, ground_truth_path):
        #Compute the IoU for all classes
        prediction, ground_truth = self.parse_sample(prediction_path, ground_truth_path)
        class_iou_mapping = {}
        non_zero_iou_results = []
        for target_class_ids in range(self.num_classes):
            m = tf.keras.metrics.IoU(num_classes=self.num_classes, target_class_ids=[target_class_ids])
            m.update_state(ground_truth, prediction)
            iou = m.result().numpy()
            class_name = self.class_names[target_class_ids]
            class_iou_mapping[class_name] = iou
            if iou > 0.0: # Only append IoUs that are greater than 0 for mean calculation
                non_zero_iou_results.append(iou)
                
        miou = np.sum(non_zero_iou_results)/len(non_zero_iou_results) if non_zero_iou_results else None
        return class_iou_mapping, miou
    
    def print_class_ious_miou(self, class_iou_mapping, miou):
        # Find the maximum length of class name for alignment
        max_length_class_name = max(len(class_name) for class_name in class_iou_mapping.keys())
        # Print header
        print(f"{'Class Name':<{max_length_class_name}} | {'IoU'}")
        print('-' * (max_length_class_name + 10))
        # Print each class's IoU
        for class_name, iou in class_iou_mapping.items():
            print(f"{class_name:<{max_length_class_name}} | {iou}")
        # Print the mean IoU (MIoU)
        if miou is not None:
            print("\nMIoU = " + str(miou))
        else:
            print("No valid IoU results found")
            
            
    def compute_average_miou(self, ground_truth_images_path, ipm_files_path, stitched_images_path):
         # Compute average IoU for IPM vs Ground Truth and Stitched Image vs Ground Truth
            
        ground_truth_images_path = sorted(glob.glob(os.path.join(ground_truth_images_path, "*.png")))
        ipm_files_path = sorted(glob.glob(os.path.join(ipm_files_path, "*.npz")))
        stitched_images_path = sorted(glob.glob(os.path.join(stitched_images_path, "*.png")))

        image_number_diff = len(ground_truth_images_path) - len(stitched_images_path)


        average_class_iou_mapping_ipm = {'Class 0 - Terrain/Ground': 0.0,
                                     'Class 1 - Road': 0.0,
                                     'Class 2 - Sidewalk/Parking': 0.0,
                                     'Class 3 - Building/Obstacle': 0.0,
                                     'Class 4 - Vegetation': 0.0,
                                     'Class 5 - Person': 0.0,
                                     'Class 6 - Rider/Motorcycle': 0.0,
                                     'Class 7 - Car/Truck/Bus/Van': 0.0,
                                     'Class 8 - Sky': 0.0}

        average_class_iou_mapping_stitched = {'Class 0 - Terrain/Ground': 0.0,
                                            'Class 1 - Road': 0.0,
                                            'Class 2 - Sidewalk/Parking': 0.0,
                                            'Class 3 - Building/Obstacle': 0.0,
                                            'Class 4 - Vegetation': 0.0,
                                            'Class 5 - Person': 0.0,
                                            'Class 6 - Rider/Motorcycle': 0.0,
                                            'Class 7 - Car/Truck/Bus/Van': 0.0,
                                            'Class 8 - Sky': 0.0}

        average_miou_ipm = 0.0
        average_miou_stitched = 0.0

        ipm_counter = [0] * 9
        stitched_counter = [0] * 9

        for i in range(len(stitched_images_path)):

            print(" processing image " + str(i+1) + " out of " + str(len(stitched_images_path)))
            
            # get matching image data
            ground_truth_path = ground_truth_images_path[i + image_number_diff]
            stitched_path = stitched_images_path[i]
            
            ground_truth_image = tf.image.decode_png(tf.io.read_file(ground_truth_path), channels=3)
            stitched_image = tf.image.decode_png(tf.io.read_file(stitched_path), channels=3)
            
            ipm_array = self.get_image_from_file(ipm_files_path[i + image_number_diff])
            ipm_image = tf.convert_to_tensor(ipm_array) 
           
            class_iou_mapping_ipm, miou_ipm = self.compute_class_ious_miou(ipm_image, ground_truth_image)
            for ctr, (key, value) in zip(range(len(class_iou_mapping_ipm.items())), class_iou_mapping_ipm.items()):
                average_class_iou_mapping_ipm[key] += value
                if value > 0.0:
                    ipm_counter[ctr] += 1
            average_miou_ipm += miou_ipm
            
            class_iou_mapping_stitched, miou_stitched = self.compute_class_ious_miou(stitched_image, ground_truth_image)
            for ctr, (key, value) in zip(range(len(class_iou_mapping_stitched.items())), class_iou_mapping_stitched.items()):
                average_class_iou_mapping_stitched[key] += value
                if value > 0.0:
                    stitched_counter[ctr] += 1
            average_miou_stitched += miou_stitched

            clear_output(wait=True)

        for ctr, (key, value) in zip(range(len(class_iou_mapping_ipm.items())), average_class_iou_mapping_ipm.items()):
            if ipm_counter[ctr] != 0.0:
                average_class_iou_mapping_ipm[key] /= ipm_counter[ctr]

        average_miou_ipm /= len(stitched_images_path)

        for ctr, (key, value) in zip(range(len(class_iou_mapping_stitched.items())), average_class_iou_mapping_stitched.items()):
            if stitched_counter[ctr] != 0.0:
                average_class_iou_mapping_stitched[key] /= stitched_counter[ctr]

        average_miou_stitched /= len(stitched_images_path)

        print("IPM vs Ground Truth")
        print("-------------------------------------")
        self.print_class_ious_miou(average_class_iou_mapping_ipm, average_miou_ipm)

        print("-------------------------------------")

        print("Stitched Image vs Ground Truth")
        print("-------------------------------------")
        self.print_class_ious_miou(average_class_iou_mapping_stitched, average_miou_stitched)

        
    def create_binary_segmentation_maps(self, label_map, class_labels):
        #Create binary segmentation maps for each class.
        binary_maps = [(label_map == class_label) for class_label in class_labels]
        return binary_maps
    
    
    def plot_binary_segmentation_with_ious(self, prediction_path, ground_truth_path):
        class_labels = list(self.rgb_to_class_id.values())
        prediction, ground_truth = self.parse_sample(prediction_path, ground_truth_path)
        y_true_binary_maps = self.create_binary_segmentation_maps(ground_truth, class_labels)
        y_pred_binary_maps = self.create_binary_segmentation_maps(prediction, class_labels)

        # Compute IoUs
        class_iou_mapping, miou = self.compute_class_ious_miou(prediction_path, ground_truth_path)
        all_iou_results = list(class_iou_mapping.values())

        # Determine the layout
        num_classes = len(class_labels)
        max_rows = (num_classes + 1) // 2  # Calculate the number of rows based on the number of classes

        columns = 4  # Four columns for Ground Truth and Prediction (twice each)

        # Increase the plot size
        fig, axes = plt.subplots(max_rows, columns, figsize=(22, 6 * max_rows))

        # Populate the subplots
        for class_number, (true_map, pred_map, iou) in enumerate(zip(y_true_binary_maps, y_pred_binary_maps, all_iou_results)):
            row = class_number // (columns // 2)
            col = (class_number % (columns // 2)) * 2  # Alternating columns for ground truth and prediction (switched positions)
            class_name = self.class_names[class_number]

            # Prediction
            axes[row, col].imshow(pred_map)
            axes[row, col].set_title(f"Prediction {class_name} - IoU: {iou:.4f}", fontdict={'fontsize': 16})
            axes[row, col].axis('off')  # Hide axes

            # Ground Truth
            axes[row, col + 1].imshow(true_map)
            axes[row, col + 1].set_title(f"Ground Truth {class_name}", fontdict={'fontsize': 16})
            axes[row, col + 1].axis('off')  # Hide axes

        # Complete segmentation map with Mean IoU
        row = max_rows - 1
        axes[row, 2].imshow(prediction)
        axes[row, 2].set_title(f"Complete Prediction - MIoU: {miou:.4f}", fontdict={'fontsize': 16})
        axes[row, 2].axis('off')

        axes[row, 3].imshow(ground_truth)
        axes[row, 3].set_title(f"Complete Ground Truth", fontdict={'fontsize': 16})
        axes[row, 3].axis('off')

        axes[row, 2].axis('off')
        axes[row, 3].axis('off')

        # Show the plot
        plt.tight_layout()
        plt.show()