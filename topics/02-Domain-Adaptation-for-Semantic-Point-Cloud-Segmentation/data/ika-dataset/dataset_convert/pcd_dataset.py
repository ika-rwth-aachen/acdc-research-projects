#!/usr/bin/env python3
import os
import glob
import numpy as np

import argparse
import tqdm
from pyntcloud import PyntCloud

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
    12: 0,  # drivable surface

    # Sidewalk
    13: 1,  # sidewalk
    14: 1,  # parking

    # Building
    10: 2,  # obstacle

    # Pole
    11: 3,  # traffic control

    # Vegetation
    15: 4,  # vegetation
    16: 4,  # flat terrain

    # Person
    7: 5,  # person

    # Two-wheeler
    5: 6,  # motor cycle
    6: 6,  # bicycle
    8: 6,  # rider

    # Car
    1: 7,  # car

    # Truck
    2: 8,  # truck
    4: 8,  # trailer

    # Bus
    3: 9,  # bus

    # None
    0: 10,  # noise
    9: 10  # animal
}

if __name__ == '__main__':
    parser = argparse.ArgumentParser("./pcd_dataset.py")
    parser.add_argument(
        '--dataset', '-d',
        type=str,
        required=True,
        help='path to the directory which contains the pcd files. No Default',
    )
    parser.add_argument(
        '--output_dir', '-o',
        type=str,
        required=True,
        help='output path where the npy files should be written to. No Default',
    )
    parser.add_argument(
        '--plot', '-p',
        action='store_true',
        help='Plot the semantic image of the PCL projection',
    )

    FLAGS, unparsed = parser.parse_known_args()

    H = 32
    W = 1024

    nclasses = len(classid_to_color)  # number of classes
    laser_scan = SemLaserScan(nclasses,
                              classid_to_color,
                              project=True,
                              H=H, W=W,
                              fov_up=None,
                              fov_down=None,
                              use_ring_projection=True)

    # class reduction
    vfunc = np.vectorize(label_map.get)

    list_of_files = sorted(glob.glob(os.path.join(FLAGS.dataset, "*.pcd")))

    for index, pcd_file in tqdm.tqdm(enumerate(list_of_files)):
        cloud = PyntCloud.from_file(pcd_file)

        points = np.array(cloud.points)

        laser_scan.set_points(points=points[:, 0:3], remissions=points[:, 3], ring_index=points[:, 4].astype(np.int32))
        laser_scan.set_label(label=points[:, 6].astype(np.int32))

        if index == 0 and FLAGS.plot:
            plt.figure(figsize=(30, 3))
            plt.imshow(laser_scan.proj_sem_color)
            plt.tight_layout()
            plt.show()

        mask = laser_scan.proj_range > 0  # check if the projected depth is positive
        laser_scan.proj_range[~mask] = 0.0
        laser_scan.proj_xyz[~mask] = 0.0
        laser_scan.proj_remission[~mask] = 0.0
        laser_scan.proj_sem_label = vfunc(laser_scan.proj_sem_label)  # apply class ID mapping

        # create the final data sample with shape (32, 1024, 6)
        final_data = np.concatenate([laser_scan.proj_xyz,
                                     laser_scan.proj_remission.reshape((H, W, 1)),
                                     laser_scan.proj_range.reshape((H, W, 1)),
                                     laser_scan.proj_sem_label.reshape((H, W, 1))],
                                    axis=2)

        np.save(os.path.join(FLAGS.output_dir, os.path.basename(pcd_file).split(".")[0]), final_data)
