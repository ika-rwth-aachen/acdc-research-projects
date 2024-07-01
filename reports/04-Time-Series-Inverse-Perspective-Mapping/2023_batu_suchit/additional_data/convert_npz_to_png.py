#!/usr/bin/env python

import os
import glob
import sys
import numpy as np
from PIL import Image
from IPython.display import clear_output
from matplotlib import pyplot as plt

class File_to_Image:
    
    def __init__(self):
    
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
        rgb_encoding = np.zeros([segmentation_map.shape[0], segmentation_map.shape[1], 3], dtype=np.uint8)
        for color, class_id in self.rgb_to_class_id.items():
            rgb_encoding[segmentation_map==class_id] = color
        return rgb_encoding
    
    def get_converted_image(self, file):
        data = np.load(file)['data']
        fractional, integral = np.modf(data[...,0])
        data[...,0][fractional==0] -= 1
        data = data.astype(int)
        data = data[:,:,0]
        data = self.segmentation_map_to_rgb_encoding(data)
        return data
        
    def store_images(self, file_path, output_path):
        file_path = sorted(glob.glob(os.path.join(file_path, "*.npz")))
        for file,i in zip(file_path, range(len(file_path))):
            print("converting file " + str(i+1) + " out of " + str(len(file_path)))
            image_data = self.get_converted_image(file)
            output_filename = file.split('/')[-1]
            output_filename = output_filename.split('.')[0]
            output_filename = output_path + '/' + output_filename + '.png'
            image = Image.fromarray(image_data)
            image.save(output_filename)
            clear_output(wait=True)
        clear_output(wait=True)
        print("Images placed in " + output_path)
        
    def print_converted_image(self, file):
        image = self.get_converted_image(file)
        plt.imshow(image)