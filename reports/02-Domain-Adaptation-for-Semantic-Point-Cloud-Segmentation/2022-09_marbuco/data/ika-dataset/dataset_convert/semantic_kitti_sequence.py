#!/usr/bin/env python3
import os

import argparse
import numpy as np
import yaml
import tqdm

import matplotlib.pyplot as plt

from laserscan_semantic_kitti import SemLaserScan

label_map_to_ika = {
  # road
  60: 0,
  40: 0,
  44: 0,

  # sidewalk
  48: 1,

  # building
  50: 2,
  51: 2,
  52: 2,

  # pole
  80: 3,
  81: 3,

  # vegetation
  70: 4,
  71: 4,
  72: 4,

  # person
  30: 5,
  254: 5,
  32: 5,
  31: 5,

  # two-wheeler
  11: 6,
  15: 6,
  253: 6,
  255: 6,

  # car
  252: 7,
  20: 7,
  259: 7,
  10: 7,

  # truck
  18: 8,
  258: 8,

  # bus
  13: 9,
  257: 9,

  # none
  0: 10,
  1: 10,
  256: 10,
  16: 10,
  49: 10,
  99: 10
}

label_map = {
  0: 0,  # "unlabeled"
  1: 0,  # "outlier" mapped to "unlabeled" --------------------------mapped
  10: 1,  # "car"
  11: 2,  # "bicycle"
  13: 5,  # "bus" mapped to "other-vehicle" --------------------------mapped
  15: 3,  # "motorcycle"
  16: 5,  # "on-rails" mapped to "other-vehicle" ---------------------mapped
  18: 4,  # "truck"
  20: 5,  # "other-vehicle"
  30: 6,  # "person"
  31: 7,  # "bicyclist"
  32: 8,  # "motorcyclist"
  40: 9,  # "road"
  44: 10,  # "parking"
  48: 11,  # "sidewalk"
  49: 12,  # "other-ground"
  50: 13,  # "building"
  51: 14,  # "fence"
  52: 0,  # "other-structure" mapped to "unlabeled" ------------------mapped
  60: 9,  # "lane-marking" to "road" ---------------------------------mapped
  70: 15,  # "vegetation"
  71: 16,  # "trunk"
  72: 17,  # "terrain"
  80: 18,  # "pole"
  81: 19,  # "traffic-sign"
  99: 0,  # "other-object" to "unlabeled" ----------------------------mapped
  252: 1,  # "moving-car" to "car" ------------------------------------mapped
  253: 7,  # "moving-bicyclist" to "bicyclist" ------------------------mapped
  254: 6,  # "moving-person" to "person" ------------------------------mapped
  255: 8,  # "moving-motorcyclist" to "motorcyclist" ------------------mapped
  256: 5,  # "moving-on-rails" mapped to "other-vehicle" --------------mapped
  257: 5,  # "moving-bus" mapped to "other-vehicle" -------------------mapped
  258: 4,  # "moving-truck" to "truck" --------------------------------mapped
  259: 5,  # "moving-other"-vehicle to "other-vehicle" ----------------mapped
}

if __name__ == '__main__':
    parser = argparse.ArgumentParser("./semantic_kitti_sequence.py")
    parser.add_argument(
      '--dataset', '-d',
      type=str,
      required=True,
      help='path to the kitti dataset where the `sequences` directory is. No Default',
    )
    parser.add_argument(
      '--sequence', '-s',
      type=str,
      default="00",
      required=False,
      help='sequence to project. Defaults to %(default)s',
    )
    parser.add_argument(
      '--output_dir', '-p',
      type=str,
      required=True,
      help='path to the PCL segmentation repository. No Default',
    )
    parser.add_argument(
      '-v',
      action='store_true',
      help='produce data samples for validation set',
    )
    
    FLAGS, unparsed = parser.parse_known_args()
    
    config = "semantic-kitti.yaml"  # configuration file
    CFG = yaml.safe_load(open(config, 'r'))

    scan_paths = os.path.join(FLAGS.dataset, "sequences", FLAGS.sequence, "velodyne") # path which contains the pointclouds for each scan 

    if os.path.isdir(scan_paths):
        print("Sequence folder exists! Using sequence from %s" % scan_paths)
    else:
        print("Sequence folder doesn't exist! Exiting...")
        quit()
    
    scan_names = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(scan_paths)) for f in fn]
    scan_names.sort()
    print(len(scan_names))

    label_paths = os.path.join(FLAGS.dataset, "sequences", FLAGS.sequence, "labels") # path which contains the labels for each scan
    if os.path.isdir(label_paths):
        print("Labels folder exists! Using labels from %s" % label_paths)
    else:
        print("Labels folder doesn't exist! Exiting...")
        quit()

    label_names = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(label_paths)) for f in fn]
    label_names.sort()
    print(len(label_names))
    
    color_dict = CFG["color_map"]  # maps numeric labels in .label file to a bgr color
    nclasses = len(color_dict)     # number of classes 
    laser_scan = SemLaserScan(nclasses, color_dict, project=True)  # create a scan

    vfunc = np.vectorize(label_map.get)

    for index, (scan, label) in tqdm.tqdm(enumerate(zip(scan_names, label_names)), total=len(scan_names)):
        laser_scan.open_scan(scan)
        laser_scan.open_label(label)

        if index == 0:
            plt.figure(figsize=(30, 3))
            plt.imshow(laser_scan.proj_sem_color)
            plt.tight_layout()
            plt.show()

        mask = laser_scan.proj_range > 0   # check if the projected depth is positive 
        laser_scan.proj_range[~mask] = 0.0  
        laser_scan.proj_xyz[~mask] = 0.0
        laser_scan.proj_remission[~mask] = 0.0
        laser_scan.proj_sem_label = vfunc(laser_scan.proj_sem_label)  # map class labels to values between 0 and 33

        # create the final data sample with shape (64, 1024, 6)
        final_data = np.concatenate([laser_scan.proj_xyz,
                                     laser_scan.proj_remission.reshape((64, 1024, 1)),
                                     laser_scan.proj_range.reshape((64, 1024, 1)),
                                     laser_scan.proj_sem_label.reshape((64, 1024, 1))], axis=2)
        
        if not FLAGS.v:
            # save as npy file to the training set
            np.save(os.path.join(FLAGS.output_dir, "converted_dataset/train/" + str(index)), final_data)
        else:
            # save as npy file to the validation set
            np.save(os.path.join(FLAGS.output_dir + "converted_dataset/val/" + str(index)), final_data)
            

