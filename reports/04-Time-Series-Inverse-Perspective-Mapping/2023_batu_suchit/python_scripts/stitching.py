#!/usr/bin/env python

import os
import sys
import cv2
import matplotlib.pyplot as plt
from matplotlib.pyplot import imshow
import numpy as np

import random
import json
from PIL import Image
from scipy.stats import mode

from IPython.display import clear_output

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf


class Stitch_Images:
    
    def __init__(self,images_path, ego_motion_path, output_path, number_images_before, number_images_after, number_of_classes, wt_decay):
        self.images_path = images_path
        self.ego_motion_path = ego_motion_path
        self.output_path = output_path
        self.number_images_before = number_images_before
        self.number_images_after = number_images_after
        self.number_of_classes = number_of_classes
        self.wt_decay = min(abs(wt_decay), 1.0)
        
        self.meter_to_pixel = 14.620000000000001
        self.center_x = 1024//2
        self.center_y = 512//2
        
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
    
    # Convert secmentation map encoding to an rgb image for storing and dispaly
    def segmentation_map_to_rgb_encoding(self, segmentation_map):
        rgb_encoding = np.zeros([segmentation_map.shape[0], segmentation_map.shape[1], 3], dtype=np.uint8)
        for color, class_id in self.rgb_to_class_id.items():
            rgb_encoding[segmentation_map==class_id] = color
        return rgb_encoding
    
    # Ego-vehicle motion compensation algorithm 
    def get_affine_transform(self, ref_ego_motion, cur_ego_motion):
        translation_x = (ref_ego_motion['state']['mean'][1] - cur_ego_motion['state']['mean'][1]) * self.meter_to_pixel
        translation_y = 0.0   # since car cannot move in y-direction (scleronomic constraint) 
        heading = ref_ego_motion['state']['mean'][5] - cur_ego_motion['state']['mean'][5]
        transformation_matrix = cv2.getRotationMatrix2D((self.center_x, self.center_y), np.degrees(-heading), 1.0)
        transformation_matrix[:, 2] += [translation_x, translation_y]
        return transformation_matrix
 
    # Warp all images using Ego-vehicle compensation to reference image frame
    def warp_image(self, source_image, target_image, transformation_matrix):
        target_height, target_width = target_image.shape[:2]
        warped_image = cv2.warpAffine(source_image, transformation_matrix, (target_width, target_height))
        black_pixels = (warped_image == [0, 0, 0]).all(axis=2)
        warped_image[black_pixels] = target_image[black_pixels]
        return warped_image

    # actual image stitching algorithm to get image pixel values over temporal stacked images
    def image_stitching(self, image_stacked, ref_number):
        
        input_array = image_stacked.reshape(image_stacked.shape[0], image_stacked.shape[1],   image_stacked.shape[2]*image_stacked.shape[3])
        class_score = np.zeros((image_stacked.shape[0], image_stacked.shape[1], self.number_of_classes))
        
        fractional, integral = np.modf(input_array)
        integral = integral.astype(int)
        
        #in case of full confidence (fractional=0.0), np rounds up to next integer value hence class_label is shifted to +1
        integral_temp = integral.copy()
        integral_temp[fractional==0.0] -= 1
        integral[..., 0:image_stacked.shape[3]] = integral_temp[..., 0:image_stacked.shape[3]]
        
        #in case of full confidence (fractional=0.0), probability is actually 1.0
        fractional_temp = fractional.copy()
        fractional_temp[fractional==0.0] = 1.0
        fractional[..., 0:image_stacked.shape[3]] = fractional_temp[..., 0:image_stacked.shape[3]]
        
        class_label = np.zeros((input_array.shape[0], input_array.shape[1], input_array.shape[2]), dtype=np.int32)
        class_probability = np.zeros((input_array.shape[0], input_array.shape[1], input_array.shape[2]))
        
        for i in range(input_array.shape[2]):
            exponent = abs(ref_number - i%image_stacked.shape[3])
            class_label[:,:,i] = integral[:,:,i]
            class_probability[:,:,i] = fractional[:,:,i] * (self.wt_decay**exponent)

            for j in range(self.number_of_classes):
                class_score[:, :, j] = np.where(j == class_label[:,:,i],
                                                class_score[:, :, j] + class_probability[:,:, i], 
                                                class_score[:, :, j]) 
        
        output_array = np.argmax(class_score, axis=-1)     
        
        return output_array
 
    
    def main(self):
        
        start_image_number = self.number_images_before
        stop_image_number = len(self.images_path) - self.number_images_after
        image_stack_length = self.number_images_before + self.number_images_after + 1
        
        #iterate over all reference image frames
        for i in range(start_image_number, stop_image_number):
            ref_image = np.load(self.images_path[i])['data']
            ref_ego_motion = json.load(open(self.ego_motion_path[i]))
            image_stacked = np.empty((ref_image.shape[0], ref_image.shape[1], ref_image.shape[2], image_stack_length)
                                    , dtype=np.float32)
            count = 0
            
            #transform desired temporal images to reference image frame and stack these images 
            for j in range((i-self.number_images_before) , (i+self.number_images_after+1)):
                cur_image =  np.load(self.images_path[j])['data']
                cur_ego_motion = json.load(open(self.ego_motion_path[j]))
                transformation_matrix = self.get_affine_transform(ref_ego_motion, cur_ego_motion)
                warped_image = self.warp_image(cur_image, ref_image, transformation_matrix)
                image_stacked[:,:,:,count] = warped_image
                if i==j:
                    ref_number = count
                count = count+1
            
            #apply stitching operation to temporally stacked image
            print("stitching image " + str(i-start_image_number+1) + " out of " + str(stop_image_number-start_image_number))
            output_image = self.image_stitching(image_stacked, ref_number)
            
            #output file name same as input file name
            output_filename = self.images_path[i].split('/')[-1]
            output_filename = output_filename.split('.')[0]
            output_filename = self.output_path + '/' + output_filename + '.png'
            
            #save output image
            image_rgb = self.segmentation_map_to_rgb_encoding(output_image)
            image = Image.fromarray(image_rgb)
            image.save(output_filename)
            
            clear_output(wait=True)