## ika Dataset for Semantic Point Cloud Segmentation

### :arrow_forward: [Download](https://rwth-aachen.sciebo.de/s/lDFnC9XVlBTH9Kw)

### Folder Structure

Semantically segmented point clouds are placed in `segmentation`, pre-processed point clouds for use with the training pipeline presented in ACDC Course are placed in `segmentation_npy`.

```
ika-dataset
├── segmentation
│   ├── 1558953854504844.pcd
│   ├── 1558953858512698.pcd
│   └── ...
└── segmentation_npy
    ├── 1558953854504844.npy
    ├── 1558953858512698.npy
    └── ...
```

### PCD-Files

The .pcd-files in `segmentation` use the standard .pcd-fileformat for storing 3D point clouds. These can be read into Python using an appropriate library, e.g., [pyntcloud](https://github.com/daavoo/pyntcloud). The provided point clouds have fields *x*, *y*, *z*, *intensity*, *ring*, and, most importantly for this task, *class*. The *class* field includes an integer class ID for each point.

The classes are defined as shown below:

```py
CLASS_IDS = {
    "unknown": 0,
    "car": 1,
    "truck": 2,
    "bus": 3,
    "trailer": 4,
    "motorcycle": 5,
    "bicycle": 6,
    "pedestrian": 7,
    "rider": 8,
    "animal": 9,
    "obstacle": 10,
    "traffic_control": 11,
    "road": 12,
    "sidewalk": 13,
    "parking": 14,
    "vegetation": 15,
    "ground": 16,
}
```

### NPY-Files

The .npy-files in `segmentation_npy` have already been pre-processed to the .npy-format required for the training pipeline presented in [ACDC Exercise: Semantic Point Cloud Segmentation](https://git.rwth-aachen.de/ika/acdc-notebooks/-/blob/main/section_3_sensor_data_processing/3_semantic_pcl_segmentation_solution.ipynb).

As part of the pre-processing, the semantic classes from above have been merged according to the following mapping between class IDs. The merged class IDs match the ones used in the exercise.

```py
CLASS_ID_TO_MERGED_CLASS_ID = {
    12: 0, # road            -> road
    13: 1, # sidewalk        -> sidewalk
    14: 1, # parking         -> sidewalk
    10: 2, # obstacle        -> building
    11: 3, # traffic_control -> pole
    15: 4, # vegetation      -> vegetation
    16: 4, # ground          -> vegetation
    7: 5,  # person          -> person
    5: 6,  # motorcycle      -> two-wheeler
    6: 6,  # bicycle         -> two-wheeler
    8: 6,  # rider           -> two-wheeler
    1: 7,  # car             -> car
    2: 8,  # truck           -> truck
    4: 8,  # trailer         -> truck
    3: 9,  # bus             -> bus
    0: 10, # unknown         -> none
    9: 10, # animal          -> none
}
```

#### Pre-processing Scripts

The pre-processing was performed using the provided script [`pcd_dataset.py`](dataset_convert/pcd_dataset.py). Note that the provided directory [`dataset_convert`](dataset_convert) also contains existing pre-processing scripts for the *NuScenes* and *SemanticKITTI* datasets. These may be helpful when trying to pre-process the data samples from the datasets that you select in the research project.
