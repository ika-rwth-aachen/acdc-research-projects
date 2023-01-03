![](assets/header-logo.png)

# ACDC Research Projects

> **Note**
> This repository belongs to the MOOC [Automated and Connected Driving Challenges (ACDC)](https://www.edx.org/course/automated-and-connected-driving-challenges). It contains multiple code and data to work on multiple research projects. Enroll in the MOOC to get access to background material.

## Cloning

When cloning the repository, make sure to recursively clone the required submodules. Additionally, [Git LFS](https://git-lfs.github.com/) is used to track large files such as images. Verify that Git LFS is installed before cloning.

```bash
git lfs install
git clone --recurse-submodules https://github.com/ika-rwth-aachen/acdc-research-projects.git
```

## Where to Work

TODO

## How to start JupyterLab

TODO

## Research Topics

| No. | Topic | Task | Relevant ACDC Sections |
| --- | --- | --- | --- |
| 01 | [Domain Adaptation for Semantic Image Segmentation](topics/01-Domain-Adaptation-for-Semantic-Image-Segmentation/task.ipynb) | Develop a methodology which optimizes a neural network for semantic image segmentation trained on public datasets with regard to its predictive performance on data affected by domain shift. | Image Segmentation |
| 02 | [Domain Adaptation for Semantic Point Cloud Segmentation](topics/02-Domain-Adaptation-for-Semantic-Point-Cloud-Segmentation/task.ipynb) | Develop a methodology which optimizes a neural network for semantic point cloud segmentation trained on public datasets with regard to its predictive performance on data affected by domain shift. | Point Cloud Segmentation |
| 03 | [Domain Adaptation for Lidar Object Detection](topics/03-Domain-Adaptation-for-Lidar-Object-Detection/task.ipynb) | Develop a methodology which optimizes a neural network for lidar object detection trained on public datasets with regard to its predictive performance on data affected by domain shift. | Object Detection |
| 04 | [Time-Series Inverse Perspective Mapping](topics/04-Time-Series-Inverse-Perspective-Mapping/task.ipynb) | Develop a methodology to fuse camera image information from multiple consecutive time steps in order to compute an advanced semantic grid map using the geometry-based Inverse Perspective Mapping (IPM) approach. | Image Segmentation, Camera-based Semantic Grid Mapping |
| 05 | [Cross-Modal Depth Estimation for Traffic Light Detection](topics/06-Cross-Modal-Depth-Estimation-for-Traffic-Light-Detection/task.ipynb) | Develop a methodology for estimating the 3D position of a traffic light, given a binary image segmentation mask and a lidar point cloud. | Image Segmentation, Camera-based Semantic Grid Mapping |
| 06 | [Processing of Traffic Light Status Information in MPC-Planner](topics/10-Processing-of-Traffic-Light-Status-Information-in-MPC-Planner/task.ipynb) | Identify and implement MPC-planner functionalities to improve trajectory planning at traffic lights. | Vehicle Guidance, Connected Driving |
| 07 | [Cloud-Based Neural Network Inference](topics/11-Cloud-Based-Neural-Network-Inference/task.ipynb) | Implement and evaluate two different methodologies to moving neural network inference from automated vehicles to connected cloud servers. | Image Segmentation, Connected Driving |
