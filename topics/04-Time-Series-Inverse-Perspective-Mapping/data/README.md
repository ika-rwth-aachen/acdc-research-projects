## Synthetic Dataset for Inverse Perspective Mapping

### :arrow_forward: [Download](https://rwth-aachen.sciebo.de/s/7wfD0PZ2UasV1YA)

### Dataset Contents

Two synthetic datasets are provided, one in a static world (no dynamic traffic, only movement by ego vehicle), one in a dynamic world (including other traffic participants).

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
