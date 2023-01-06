## Bag Files for Cross-Model Depth Estimation for Traffic Light Detection

### :arrow_forward: [Download](https://rwth-aachen.sciebo.de/s/3YkCXmau4dEUZtd)

### Available Data

The provided data stems from two bag files that are also included. `scenario_traffic-lights_1` should be the default scenario to use. `scenario_traffic-lights_2` contains data for a different color-coding setup, where also the posts of traffic lights are color-coded. The camera images, semantic segmentation images, and point clouds are also provided as separate files. The provided camera information contains intrinsic camera matrices, the provided transform information contains the relations between different coordinate frames. All information was exported from the two bag files.

```
data/
├── bags/
│   ├── scenario_traffic-lights_1.bag
│   └── scenario_traffic-lights_2.bag
├── scenario_traffic-lights_1/
│   ├── camera_info.yml
│   ├── transforms.json
│   ├── camera/
│   ├── lidar/
│   └── semantic_segmentation/
└── scenario_traffic-lights_2/
    ├── camera_info.yml
    ├── transforms.json
    ├── camera/
    ├── lidar/
    └── semantic_segmentation/
```

### Rviz Visualization

For a first look at the available data, it is recommended to visualize the bag files with *Rviz*. A suitable configuration file is provided as [`visualization.rviz`](visualization.rviz).

```
rviz -d visualization.rviz
```
