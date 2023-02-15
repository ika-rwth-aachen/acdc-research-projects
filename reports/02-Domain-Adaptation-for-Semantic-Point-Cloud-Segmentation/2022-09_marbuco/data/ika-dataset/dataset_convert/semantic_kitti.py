#!/usr/bin/env python3
import os

import argparse
import numpy as np
import tqdm
import yaml

import matplotlib.pyplot as plt

from laserscan_semantic_kitti import SemLaserScan

# Layer extraction for VLP32-C
layers = np.arange(16, 48)

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
  parser = argparse.ArgumentParser("./semantic_kitti.py")
  parser.add_argument(
    '--dataset', '-d',
    type=str,
    required=True,
    help='path to the kitti dataset where the `sequences` directory is. No Default',
  )
  parser.add_argument(
    '--output_dir', '-p',
    type=str,
    required=True,
    help='output path to the PCL segmentation repository. No Default',
  )
  parser.add_argument(
    '-n', type=int, default=23201,
    help='total number of samples in training and validation sets. Maximum is 23201. Default is 23201'
  )
  parser.add_argument(
    '-s', type=float, default=0.9,
    help='split percentage of samples for training and validation sets. It should be between 0 and 1. Default is 0.1.'
  )
  parser.add_argument(
    '-v',
    action='store_true',
    help='use only 32 layers of KITTI lidar data in order to match a VLP32 laser scanner ',
  )

  FLAGS, unparsed = parser.parse_known_args()

  config = "semantic-kitti.yaml"  # configuration file

  # open config file
  print("Opening config file %s" % config)
  CFG = yaml.safe_load(open(config, 'r'))

  sequences = ["00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10"]

  scans = []
  labels = []

  for sequence in sequences:
    # path which contains the pointclouds for each scan
    scan_paths = os.path.join(FLAGS.dataset, "sequences", sequence, "velodyne")

    if os.path.isdir(scan_paths):
      print("Sequence folder exists! Using sequence from %s" % scan_paths)
    else:
      print("Sequence folder doesn't exist! Exiting...")
      quit()

    scan_names = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(scan_paths)) for f in fn]
    scan_names.sort()
    scans = scans + scan_names

  for sequence in sequences:
    # path which contains the labels for each scan
    label_paths = os.path.join(FLAGS.dataset, "sequences", sequence, "labels")

    if os.path.isdir(label_paths):
      print("Labels folder exists! Using labels from %s" % label_paths)
    else:
      print("Labels folder doesn't exist! Exiting...")
      quit()

    label_names = [os.path.join(dp, f) for dp, dn, fn in os.walk(os.path.expanduser(label_paths)) for f in fn]
    label_names.sort()
    labels = labels + label_names

  print("number of scans : ", len(scans))
  print("number of labels : ", len(labels))

  print("number of total samples to be used : ", FLAGS.n)
  print("split percentage : ", FLAGS.s)

  train_set_size = int(FLAGS.n * FLAGS.s)
  val_set_size = FLAGS.n - train_set_size

  print("number of samples in training set : ", train_set_size)
  print("number of samples in validation set : ", val_set_size)

  vfunc = np.vectorize(label_map.get)

  color_dict = CFG["color_map"]  # maps numeric labels in .label file to a bgr color
  nclasses = len(color_dict)  # number of classes

  laser_scan = SemLaserScan(nclasses, color_dict, project=True)  # create a scan with all 64 layers of KITTI lidar data

  for index, (scan, label) in tqdm.tqdm(enumerate(zip(scans[:train_set_size+val_set_size],
                                                      labels[:train_set_size+val_set_size])),
                                        total=train_set_size+val_set_size):
    laser_scan.open_scan(scan)
    laser_scan.open_label(label)

    if index == 0:
      plt.figure(figsize=(30, 3))
      plt.imshow(laser_scan.proj_sem_color)
      plt.tight_layout()
      plt.show()

    mask = laser_scan.proj_range > 0  # check if the projected depth is positive
    laser_scan.proj_range[~mask] = 0.0
    laser_scan.proj_xyz[~mask] = 0.0
    laser_scan.proj_remission[~mask] = 0.0
    laser_scan.proj_sem_label = vfunc(laser_scan.proj_sem_label)  # map class labels to values between 0 and 10

    # create the final data sample with shape (64,1024,6)
    final_data = np.concatenate([laser_scan.proj_xyz,
                                 laser_scan.proj_remission.reshape((64, 1024, 1)),
                                 laser_scan.proj_range.reshape((64, 1024, 1)),
                                 laser_scan.proj_sem_label.reshape((64, 1024, 1))],
                                axis=2)

    if index < train_set_size:
      if FLAGS.v:
        vlp_32_data = final_data[layers, :, :]
        np.save(os.path.join(FLAGS.output_dir, "converted_dataset/train/", str(index)), vlp_32_data)
      else:
        np.save(os.path.join(FLAGS.output_dir, "converted_dataset/train/", str(index)), final_data)
    else:
      if FLAGS.v:
        vlp_32_data = final_data[layers, :, :]
        np.save(os.path.join(FLAGS.output_dir, "converted_dataset/val/", str(index)), vlp_32_data)
      else:
        np.save(os.path.join(FLAGS.output_dir, "converted_dataset/val/", str(index)), final_data)

