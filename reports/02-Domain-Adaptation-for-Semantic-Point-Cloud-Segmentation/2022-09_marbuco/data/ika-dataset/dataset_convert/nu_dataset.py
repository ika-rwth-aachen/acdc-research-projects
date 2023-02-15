#!/usr/bin/env python3
import os
import numpy as np

import argparse
import tqdm

from nuscenes import NuScenes
import matplotlib.pyplot as plt

from laserscan_nuscenes import SemLaserScan

classid_to_color = {  # RGB.
    0: [0, 0, 0],  # Black.
    1: [70, 130, 180],  # Steelblue
    2: [0, 0, 230],  # Blue
    3: [135, 206, 235],  # Skyblue,
    4: [100, 149, 237],  # Cornflowerblue
    5: [219, 112, 147],  # Palevioletred
    6: [0, 0, 128],  # Navy,
    7: [240, 128, 128],  # Lightcoral
    8: [138, 43, 226],  # Blueviolet
    9: [112, 128, 144],  # Slategrey
    10: [210, 105, 30],  # Chocolate
    11: [105, 105, 105],  # Dimgrey
    12: [47, 79, 79],  # Darkslategrey
    13: [188, 143, 143],  # Rosybrown
    14: [220, 20, 60],  # Crimson
    15: [255, 127, 80],  # Coral
    16: [255, 69, 0],  # Orangered
    17: [255, 158, 0],  # Orange
    18: [233, 150, 70],  # Darksalmon
    19: [255, 83, 0],
    20: [255, 215, 0],  # Gold
    21: [255, 61, 99],  # Red
    22: [255, 140, 0],  # Darkorange
    23: [255, 99, 71],  # Tomato
    24: [0, 207, 191],  # nuTonomy green
    25: [175, 0, 75],
    26: [75, 0, 75],
    27: [112, 180, 60],
    28: [222, 184, 135],  # Burlywood
    29: [255, 228, 196],  # Bisque
    30: [0, 175, 0],  # Green
    31: [255, 240, 245]
}

label_map = {
    # Road
    24: 0,  # drivable surface

    # Sidewalk
    25: 1,  # flat terrain
    26: 1,  # sidewalk

    # Building
    28: 2,  # static man made

    # Pole
    9: 3,  # road barrier
    12: 3,  # traffic cones

    # Vegetation
    30: 4,  # vegetation
    27: 4,  # flat terrain

    # Person
    2: 5,  # adult
    3: 5,  # child
    4: 5,  # construction worker
    6: 5,  # police officer

    # Two-wheeler
    21: 6,  # motor cycle
    5: 6,  # scooter or different
    13: 6,  # bicycles rack including bicycles
    14: 6,  # bicycle
    8: 6,  # wheel chair
    7: 6,  # stroller

    # Car
    20: 7,  # police vehicle
    17: 7,  # vehicle

    # Truck
    23: 8,  # truck
    18: 8,  # construction vehicles
    19: 8,  # ambulance
    22: 8,  # trailer

    # Bus
    15: 9,  # bendy bus
    16: 9,  # ridgid bus

    # None
    0: 10,  # noise
    1: 10,  # animal
    10: 10,  # debris
    11: 10,  # push pull able
    29: 10,  # points in the background
    31: 10  # ego vehicle
}

if __name__ == '__main__':
    parser = argparse.ArgumentParser("./nu_dataset.py")
    parser.add_argument(
        '--dataset', '-d',
        type=str,
        required=True,
        help='path to the nuscenes dataset where the `/data/sets/nuscenes` directory is. No Default',
    )
    parser.add_argument(
        '--output_dir', '-p',
        type=str,
        required=True,
        help='output path to the PCL segmentation repository. No Default',
    )
    parser.add_argument(
        '-v',
        action='store_true',
        help='produce data samples for validation set',
    )

    FLAGS, unparsed = parser.parse_known_args()

    H = 32
    W = 1024

    nusc = NuScenes(version='v1.0-trainval', dataroot=FLAGS.dataset, verbose=True)
    nclasses = len(classid_to_color)  # number of classes
    laser_scan = SemLaserScan(nclasses,
                              classid_to_color,
                              project=True,
                              H=H, W=W,
                              fov_up=12, fov_down=-30,
                              use_ring_projection=False)

    # class reduction
    vfunc = np.vectorize(label_map.get)

    for index, my_sample in tqdm.tqdm(enumerate(nusc.sample), total=len(nusc.sample)):
        sample_data_token = my_sample['data']['LIDAR_TOP']
        sd_record = nusc.get('sample_data', sample_data_token)
        # path to the bin file containing the x, y, z and intensity of the points in the point cloud
        pcl_path = os.path.join(nusc.dataroot, sd_record['filename'])
        # path to the bin file containing the labels of the points in the point cloud
        lidarseg_labels_filename = os.path.join(nusc.dataroot, nusc.get('lidarseg', sample_data_token)['filename'])

        laser_scan.open_scan(pcl_path)  # spherically project the point cloud scans
        laser_scan.open_label(lidarseg_labels_filename)

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

        # create the final data sample with shape (H, W, 6)
        final_data = np.concatenate([laser_scan.proj_xyz,
                                     laser_scan.proj_remission.reshape((H, W, 1)),
                                     laser_scan.proj_range.reshape((H, W, 1)),
                                     laser_scan.proj_sem_label.reshape((H, W, 1))],
                                    axis=2)

        if index < int(len(nusc.sample) * 0.75):
            np.save(os.path.join(FLAGS.output_dir, "train", str(index)), final_data)
        else:
            np.save(os.path.join(FLAGS.output_dir, "val", str(index)), final_data)
