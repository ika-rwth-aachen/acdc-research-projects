# Synthetic Datasets for Inverse Perspective Mapping

### :arrow_forward: [Download](https://rwth-aachen.sciebo.de/s/fbC6sBbs4W06b96)

## Static/Dynamic World Datasets

### Dataset Contents

Two main synthetic datasets are provided, one in a static world (no dynamic traffic, only movement by ego vehicle), one in a dynamic world (including other traffic participants).

Both datasets include data from 4 vehicle-mounted cameras (front, rear, left, right), as well as one drone camera.
For all cameras, actual images and semantic segmentation images are provided.
In addition to the camera data, ego data information including the ego vehicle's state is attached.
All subfolders contain many consecutive samples named with the sample timestamp, so corresponding samples from all data sources can be matched via filename.

All data has been exported from ROS bag files, which are provided as well.

Extra information about the camera intrinsics/extrinsics as well as the semantic segmentation coloring is also included.

```
ipm-dataset/
├── dynamic-world.bag
├── static-world.bag
├── camera_configs/
│   ├── rgb_to_class_id.py
│   ├── drone.yaml
│   ├── front.yaml
│   ├── left.yaml
│   ├── rear.yaml
│   └── right.yaml
├── dynamic-world/
│   ├── camera/
│   │   ├── drone/
│   │   ├── front/
│   │   ├── left/
│   │   ├── rear/
│   │   └── right/
│   ├── ego_data/
│   └── semantic_segmentation/
│       ├── drone/
│       ├── front/
│       ├── left/
│       ├── rear/
│       └── right/
└── static-world/
    ├── camera/
    │   ├── drone/
    │   ├── front/
    │   ├── left/
    │   ├── rear/
    │   └── right/
    ├── ego_data/
    └── semantic_segmentation/
        ├── drone/
        ├── front/
        ├── left/
        ├── rear/
        └── right/
```

### Color-Coding of Semantic Segmentation

The color-coding of semantic classes is defined in `camera_configs/rgb_to_class_id.py`.

### Camera Intrinsics/Extrinsics

Information about the camera intrinsics (e.g., focal length) and the camera extrinsics (i.e., pose) are provided in the form of yaml-files.
This format is directly compatible with the IPM implementation (`ipm.py`) that should be used, see the task description.

### Ego Data Format

Information about the ego vehicle state is provided in the form of ego data json-files.
The interesting state information is found under the `state.mean` key.

| `state.mean` index | Description | Unit |
| --- | --- | --- |
| 0 | x-position in world frame | m |
| 1 | y-position in world frame | m |
| 2 | z-position in world frame | m |
| 3 | velocity magnitude | m/s |
| 4 | acceleration magnitude | m/s^2 |
| 5 | yaw angle | rad |
| 6 | yaw rate | rad/s |
| 7 | width | m |
| 8 | length | m |
| 9 | height | m |

## Advanced Vehicle Dynamics Dataset *(optional)*

### :arrow_forward: [Download](https://rwth-aachen.sciebo.de/s/fbC6sBbs4W06b96)

### Dataset Contents

A third dataset in a dynamic world is provided that was recorded using a different simulation software. This dataset also includes information about the vehicle's roll and pitch angles due to vehicle dynamics.

The dataset includes data from 6 vehicle-mounted cameras (front center, front left, front right, rear center, rear left, rear right), as well as an orthographic drone camera.
For all cameras, actual images and semantic segmentation images are provided.
In addition to the camera data, ego data information including the ego vehicle's state is attached. Note that the format differs from datasets 1 and 2.
All images can be matched to one another via their filename. Matching images to ego data information involves the naming conventions stated further below.
Extra information about the camera intrinsics/extrinsics is also included.

```
advanced-vehicle-dynamics/
├── dynamic-world.bag
├── static-world.bag
├── camera_configs/
│   ├── front_center.yaml
│   ├── front_left.yaml
│   ├── front_right.yaml
│   ├── rear_center.yaml
│   ├── rear_left.yaml
│   ├── rear_right.yaml
├── camera/
│   ├── front_center/
│   ├── front_left/
│   ├── front_right/
│   ├── rear_center/
│   ├── rear_left/
│   └── rear_right/
├── semantic_segmentation/
│   ├── front_center/
│   ├── front_left/
│   ├── front_right/
│   ├── rear_center/
│   ├── rear_left/
│   └── rear_right/
└── odometry/
    ├── filtered_Town02_ClearNoon_6_55_2023-04-28-09-51-46.txt
    └── ...
```

### Camera Intrinsics/Extrinsics

Information about the camera intrinsics (e.g., focal length) and the camera extrinsics (i.e., pose) are provided in the form of yaml-files.
This format is directly compatible with the IPM implementation (`ipm.py`) that should be used, see the task description.

### Orthographic Drone Images

The orthographic bird's-eye-view images cover an area of 100m x 100m around the vehicle.

### Odometry

Ego data / odometry information is provided in the form of csv-files. The column headers indicate the quantities. Each timestamp corresponds to one row.

### Matching Images to Odometry

The images are named after the following naming schema

```
<town>_<weather>_<#run>_<rosbag-time>_<image-time>.png
e.g.: 00_00_00_0_2023-01-27-11-09-15_123456.png
```

with the following available town indices

```
00,Town01
01,Town02
02,Town03
04,Town04
05,Town05
10,Town10HD
```

and the following available weather indices

```
00,ClearNoon
01,ClearSunset
02,CloudyNoon
03,CloudySunset
04,SoftRainNoon
05,WetCloudyNoon
06,WetCloudySunset
07,WetNoon
08,WetSunset
09,HardRainNoon
10,ClearNight
```

This information allows matching images to one of the given odometry files. The exact odometry line can then be matched via the timestamp.
