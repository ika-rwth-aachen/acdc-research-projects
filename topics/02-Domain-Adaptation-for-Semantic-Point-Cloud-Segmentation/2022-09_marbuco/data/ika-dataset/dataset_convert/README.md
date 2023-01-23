## nuScenes dataset

This directory contains a script `nu_dataset.py` which converts pointcloud samples in [nuScenes dataset](https://www.nuscenes.org/nuscenes#lidarseg) to a data format which can be used to train the neural networks developed for PCL segmentation. Moreover, `laserscan.py` includes definations of some of the classes used in`nu_dataset.py`.

The directory should have the file structure:
```
├── ImageSet
    ├── train.txt
    ├── val.txt
    ├── test.txt
├── train
├── val
├── test
├── nu_dataset.py
├── laserscan.py

```
The data samples are located in the directories `train`, `val` and `test`. The `*.txt` files within the directory `ImageSet` contain the filenames for the corresponding samples in data directories.

### Conversion Script 

Conversion script `nu_dataset.py` uses some of the functions and classes defined in [nuScenes devkit](https://github.com/nutonomy/nuscenes-devkit) and in [API for SemanticKITTI](https://github.com/PRBonn/semantic-kitti-api#readme). It opens pointcloud scans, spherically projects the points in these scans into 2D and store these projections as `*.npy` files. Each of these `*.npy` files contains a tensor of size `32 X 1024 X 6`. The 6 channels correspond to

0. X-Coordinate in [m]
1. Y-Coordinate in [m]
2. Z-Coordinate in [m]
3. Intensity
4. Depth in [m]
5. Label ID

The script stores the projections in `nuscenes_dataset/train` or `nuscenes_dataset/val` directory of PCL segmentation repository.

```bash
./nu_dataset.py --dataset /path/to/nuScenes/dataset/ --output_dir /path/to/PCLSegmentation/ -v
```
where:
- `dataset` is the path to the nuScenes dataset where the `/data/sets/nuscenes` directory is.
- `output_dir` is the output path to the PCL segmentation repository.
- `v`is a flag. If it is used, the projections are stored in validation set, otherwise they are stored in training set.

To be able to run the script, the instructions explaining how to use nuScenes devkit and how to download the dataset can be found [here](https://github.com/nutonomy/nuscenes-devkit#nuscenes-lidarseg).


## SemanticKITTI dataset

This directory contains the `semantic_kitti.py` which converts pointcloud samples in [SemanticKITTI dataset](http://www.semantic-kitti.org/) to a data format which can be used to train the neural networks developed for PCL segmentation. It also includes a small `train` and `val` split with 20 samples and 2 samples, respectively.

### Conversion Scripts 

The scripts use some of the functions and classes defined in [API for SemanticKITTI](https://github.com/PRBonn/semantic-kitti-api#readme). They open pointcloud scans, project the points in these scans into 2D and store these projections as `*.npy` files. Each of these `*.npy` files contains a tensor of size `64 X 1024 X 6`. The 6 channels correspond to

0. X-Coordinate in [m]
1. Y-Coordinate in [m]
2. Z-Coordinate in [m]
3. Intensity (with range [0-255])
4. Depth in [m]
5. Label ID

#### Downloading SemanticKITTI dataset

To be able to run the scripts, firstly, SemanticKITTI dataset should be downloaded. Information about files in this dataset and how to download it is provided [here](http://www.semantic-kitti.org/dataset.html)

SemanticKITTI dataset is organized in the following format:

```
/kitti/dataset/
          └── sequences/
                  ├── 00/
                  │   ├── poses.txt
                  │   ├── image_2/
                  │   ├── image_3/
                  │   ├── labels/
                  │   │     ├ 000000.label
                  │   │     └ 000001.label
                  |   ├── voxels/
                  |   |     ├ 000000.bin
                  |   |     ├ 000000.label
                  |   |     ├ 000000.occluded
                  |   |     ├ 000000.invalid
                  |   |     ├ 000001.bin
                  |   |     ├ 000001.label
                  |   |     ├ 000001.occluded
                  |   |     ├ 000001.invalid
                  │   └── velodyne/
                  │         ├ 000000.bin
                  │         └ 000001.bin
                  ├── 01/
                  ├── 02/
                  .
                  .
                  .
                  └── 21/
```
#### Using API for SemanticKITTI

##### semantic_kittit_sequence.py  

The script projects the scans in the specified sequence and stores the projections in `semantic_kitti_dataset/train` or `semantic_kitti_dataset/val` directory of PCL segmentation repository.

```bash
./semantic_kitti_sequence.py --sequence 00 --dataset /path/to/kitti/dataset/ --output_dir /path/to/PCLSegmentation/ -v
```
where:
- `sequence` is the sequence to be accessed (optional, default value is 00).
- `dataset` is the path to the kitti dataset where the `sequences` directory is.
- `output_dir` is the output path to the PCL segmentation repository. 
- `v`is a flag. If it is used, the projections are stored in validation set, otherwise they are stored in training set.

##### semantic_kitti.py 

The script randomly picks a specified number of scans from all sequences and stores their projections in `semantic_kitti_dataset/train` and `semantic_kitti_dataset/val` directory of PCL segmentation repository. 

```bash
./semantic_kitti.py --dataset /path/to/kitti/dataset/ --output_dir /path/to/PCLSegmentation/ -n 20 -s 0.8 -v
```
where:
- `dataset` is the path to the kitti dataset where the `sequences` directory is.
- `output_dir` is the output path to the PCL segmentation repository. 
- `n`is the number of training samples (projections) to be used in training and validation sets. Maximum is 23201. Default is 20.
- `s`is the split ratio of samples between training and validation sets. It should be between 0 and 1. Default is 0.9.
- `v` is a flag. If it is used, the projections consist of 32 layers instead of 64. The script extracts 32 specified layers from the SemanticKITTI projections which are 64-layered.

### Generalization to VLP-32 Data

The ultimate goal is to have a network which is trained on higher resolution KITTI dataset and does semantic segmentation on VLP-32 lidar data. KITTI pointcloud projections used in training should be modified in such a way which makes them similar to VLP-32 data. One method is to extract 32 specified layers from the KITTI point cloud projections. However, the network has not been able to generalize to VLP-32 Data well yet. The tested layer configurations are as follows. 

#### Tested Layer Configurations

`layers` array, which is defined in `conversion_3.py` script, specifies 32 layers which will be extracted from KITTI projections. 

- layers = np.arange(16,48)
- configuration 3 is used, but intensity is not used as a feature in semantic segmentation.
- layers = np.concatenate([np.array([14, 15, 17, 24, 26, 30, 31, 34, 36, 37, 39, 41, 43, 45]), np.arange(46, 64)])
- layers = [0, 4, 8, 11, 12, 13, 15, 17, 19, 21, 23, 25, 27, 29, 30, 31, 32, 33, 35, 37, 39, 41, 43, 45, 47, 49, 50, 51, 52, 55, 59, 63] 
- directly projecting KITTI point clouds into 32-layered projections instead of extracting 32 layers from 64-layered projections.
- layers = np.concatenate([np.array([14, 15, 16, 17, 25, 26, 27, 31, 33, 36, 39, 41, 43, 45]), np.arange(46, 64)])
- layers = np.arange(1,64,2)


