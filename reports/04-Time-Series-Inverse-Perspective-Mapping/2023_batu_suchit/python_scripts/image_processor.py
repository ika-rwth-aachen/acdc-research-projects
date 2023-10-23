#!/usr/bin/env python

import os
import glob
import sys
import numpy as np
import cv2

from PIL import Image
from IPython.display import clear_output

# Define a class for post processing
class ImageProcessor:
    
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
    
    def segmentation_map_to_rgb_encoding(self, segmentation_map):
         # Convert segmentation map to RGB encoding
        rgb_encoding = np.zeros([segmentation_map.shape[0], segmentation_map.shape[1], 3], dtype=np.uint8)
        for color, class_id in self.rgb_to_class_id.items():
            rgb_encoding[segmentation_map==class_id] = color
        return rgb_encoding
    
    def print_image_from_file(self, ipm_file, output_image):
        img = np.load(ipm_file)['data']
        img_corrected = img
        fractional, integral = np.modf(img[...,0])
        img_corrected[...,0][fractional==0] -= 1
        img_class = img_corrected.astype(int)
        img_class_0 = img_class[:,:,0]
        img_rgb = self.segmentation_map_to_rgb_encoding(img_class_0)
        ipm_image = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
        cv2.imwrite(output_image, ipm_image)
    
    def extract_region(self, ipm_image, left_width=65, right_width=90, length=95):
        # Extract a region of the mask based on provided dimensions
        center_x = ipm_image.width // 2
        center_y = ipm_image.height // 2
        left_x = center_x - left_width
        right_x = center_x + right_width
        top_y = center_y - length // 2
        bottom_y = center_y + length // 2
        extracted_region = ipm_image.crop((left_x, top_y, right_x, bottom_y))
        return extracted_region

    def modify_blue_to_black(self, extracted_region):
        # Modify the car object (Ego Vehicle) in the extracted region to black
        extracted_region_rgb = extracted_region.convert('RGB')
        modified_extracted_region = Image.new('RGB', extracted_region_rgb.size)
        for y in range(extracted_region_rgb.height):
            for x in range(extracted_region_rgb.width):
                r, g, b = extracted_region_rgb.getpixel((x, y))
                if r == 0 and g == 0 and b == 142:  # Threshold to detect blue
                    modified_extracted_region.putpixel((x, y), (0, 0, 0))
                else:
                    modified_extracted_region.putpixel((x, y), (r, g, b))
        return modified_extracted_region

    def paste_to_center(self, stitched_image, modified_extracted_region):
        paste_center_x = stitched_image.width // 2
        paste_center_y = stitched_image.height // 2
        paste_left_x = paste_center_x - modified_extracted_region.width // 2
        paste_top_y = paste_center_y - modified_extracted_region.height // 2
        final_image = stitched_image.copy()
        final_image.paste(modified_extracted_region, (paste_left_x, paste_top_y))
        return final_image
    
    def save_final_image(self, output_path, final_image):
        final_image.save(output_path)
        
    
    def main(self, ipm_path, stitched_path, output_path):
        
        #stitched images will be fewer in number, difference required to match correct ipm and stitched image
        image_number_diff = len(ipm_path) - len(ipm_path)
        
        #get input images
        ipm_path = sorted(glob.glob(os.path.join(ipm_path, "*.npz")))
        stitched_path = sorted(glob.glob(os.path.join(stitched_path, "*.png")))
        
        for i in range(len(stitched_path)):
    
            print(" processing image " + str(i+1) + " out of " + str(len(stitched_path)))
            
            # get output file name
            output_image = stitched_path[i].split('/')[-1]
            output_image = output_path + '/' + output_image
            
            # get ipm image from file and save temporarily
            self.print_image_from_file(ipm_path[i + image_number_diff], output_image)
            ipm_image = Image.open(output_image)
            
            stitched_image = Image.open(stitched_path[i])

            # extract Ego-vehicle region in ipm image
            extracted_region = self.extract_region(ipm_image)
            
            # change extracted region color to black
            modified_extracted_region = self.modify_blue_to_black(extracted_region)
            
            # paste extracted region to center of stitched image
            final_image = self.paste_to_center(stitched_image, modified_extracted_region)
            
            # save output image in output path
            self.save_final_image(output_image, final_image)

            clear_output(wait=True)
    
