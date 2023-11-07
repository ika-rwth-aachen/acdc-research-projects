#!/usr/bin/env python

import os
import matplotlib.pyplot as plt
from matplotlib.pyplot import imshow
import numpy as np
import random
import glob

from IPython.display import clear_output

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import tensorflow as tf
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input
from tensorflow.keras.layers import Dense, Conv2D, Conv2DTranspose, MaxPooling2D
from tensorflow.keras.layers import BatchNormalization, Dropout
from tensorflow.keras.layers import Activation
from tensorflow.keras.layers import Concatenate

from PIL import Image

random.seed(123)


class SegmentationModel:
    def __init__(self, input_shape, num_classes, udepth=5, filters1=16,
                 kernel_size=(3, 3), activation=tf.nn.relu, batch_norm=True, dropout=0.1):
        self.input_shape = input_shape
        self.num_classes = num_classes
        self.udepth = udepth
        self.filters1 = filters1
        self.kernel_size = kernel_size
        self.activation = activation
        self.batch_norm = batch_norm
        self.dropout = dropout
        
        self.rgb_to_class_id = {
            (152, 251, 152): 0,
            (128, 64, 128): 1,
            (244, 35, 232): 2,
            (70, 70, 70): 3,
            (107, 142, 35): 4,
            (255, 0, 0): 5,
            (220, 20, 60): 6,
            (0, 0, 142): 7,
            (0, 145, 198): 8,
        }
        
        self.model = self.getModel()
    
    # normalize image values to get float values
    def normalize(self, image):
        return tf.cast(image, tf.float32) / 255.0
    
    # resize input image to desired shape
    def parse_sample(self, image_path):
        image_rgb = tf.image.decode_png(tf.io.read_file(image_path), channels=3)
        image_rgb = tf.image.resize(image_rgb, [self.input_shape[0], self.input_shape[1]], method=tf.image.ResizeMethod.BILINEAR)
        image_rgb = tf.cast(image_rgb, dtype=tf.uint8)
        return image_rgb
    
    # segmentation NN encoder block
    def encoder(self, input):
        t = input
        encoder_layers = self.udepth * [None]
        for d in range(self.udepth):
            filters = (2 ** d) * self.filters1
            t = Conv2D(filters=filters, kernel_size=self.kernel_size, padding="same", activation=self.activation)(t)
            t = BatchNormalization()(t) if self.batch_norm else t
            t = Conv2D(filters=filters, kernel_size=self.kernel_size, padding="same", activation=self.activation)(t)
            t = encoder_layers[d] = BatchNormalization()(t) if self.batch_norm else t
            if d < (self.udepth - 1):
                t = MaxPooling2D(pool_size=(2, 2), padding="same")(t)
                t = Dropout(rate=self.dropout)(t) if self.dropout > 0 else t
        return encoder_layers
    
    # segmentation NN decoder block
    def decoder(self, encoder_layers):
        t = encoder_layers[self.udepth - 1]
        for d in reversed(range(self.udepth - 1)):
            filters = (2 ** d) * self.filters1
            t = Conv2DTranspose(filters=filters, kernel_size=self.kernel_size, strides=(2, 2), padding="same")(t)
            t = Concatenate()([encoder_layers[d], t])
            t = Dropout(rate=self.dropout)(t) if self.dropout > 0 else t
            t = Conv2D(filters=filters, kernel_size=self.kernel_size, padding="same", activation=self.activation)(t)
            t = BatchNormalization()(t) if self.batch_norm else t
            t = Conv2D(filters=filters, kernel_size=self.kernel_size, padding="same", activation=self.activation)(t)
            t = BatchNormalization()(t) if self.batch_norm else t
        return t
    
    # NN Model structure definition
    def getModel(self):
        input_tensor = Input(self.input_shape)
        encoder_layers = self.encoder(input_tensor)
        reconstruction = self.decoder(encoder_layers)
        logits = Conv2D(filters=self.num_classes, kernel_size=self.kernel_size, padding="same",
                        activation=self.activation)(reconstruction)
        probabilities = Activation("softmax")(logits)
        return Model(inputs=input_tensor, outputs=probabilities)
    
    # load model weights
    def load_weights(self, weights_directory):
        self.model.load_weights(weights_directory)

    # segmentation prediction for input image, output is npz file with same file name
    def predict(self, images_path, output_path):
        
        # get directory name for printing
        dir_name = images_path.split("/")[-1]
        
        # get file pathes
        images_path = sorted(glob.glob(os.path.join(images_path, "*.png"))) 
        
        for i in range(len(images_path)):

            print("processing " + dir_name + " image " + str(i) + " out of " + str(len(images_path)))
            
            # get image file name as output
            file_name = images_path[i].split('/')[-1]
            file_name = file_name.split('.')[0] + '.npz'
            
            #pre process image
            image = self.parse_sample(images_path[i])
            image = self.normalize(image)
            
            # get segmentation class probabilities
            image = tf.expand_dims(image, axis=0)
            probabilities = self.model.predict(image)
            probabilities = tf.squeeze(probabilities)
            
            # prediction is taken as the top 3 class probabilities and their indices (class encoding value)
            prediction = np.flip(np.sort(probabilities), axis=-1)[:,:,0:3]
            prediction_indices = np.flip(np.argsort(probabilities), axis=-1)[:,:,0:3]
            
            # data representation contains class encoding as integer part & corresponding probability as decimal part
            value = prediction_indices + prediction
            
            # save values obtained for image as a ".npz" file
            output_file = output_path + '/' + file_name
            np.savez_compressed(output_file, data=value)
            
            clear_output(wait=True)
            
        print(dir_name + " done...")
