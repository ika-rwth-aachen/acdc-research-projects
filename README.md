![](assets/header-logo.png)

# ACDC Research Projects

> **Note**
> This repository belongs to the MOOC [Automated and Connected Driving Challenges (ACDC)](https://www.edx.org/course/automated-and-connected-driving-challenges). It contains code and data to work on multiple research projects. Enroll in the MOOC to get access to background material.

Thank you for your interest in the *ACDC - Research Projects*! Working on a research project gives you the chance to apply your newly learnt skills to actual research questions. We provide you with detailed descriptions of suggested research topics including data and suggestions for how to face the presented challenges. In the spirit of open and reproducible research, we are encouraging you to produce a research report in the form of a Jupyter Notebook, for which we also provide a template. If you choose to work on one of the challenges, you may publicly share your research so other researchers can then build upon your results. The research project gives you the opportunity to demonstrate that you are capable of solving complex problems in the field of automated and connected driving.

All research is typically a joint effort between multiple collaborators. In that spirit, we actively encourage you to work on a project and a final report together with other participants of the ACDC MOOC.

## Finished Research Projects

You can find all published research reports in the [`reports/`](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/reports) folder. It can also be interesting to take a look at the [forks of the repository](https://github.com/ika-rwth-aachen/acdc-research-projects/network/members) to check out what everyone else is working on.

- [Domain Adaptation for Semantic Point Cloud Segmentation](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/reports/02-Domain-Adaptation-for-Semantic-Point-Cloud-Segmentation/2022-09_marbuco/report.ipynb) by [@marbuco](https://github.com/marbuco/) (09/2022)
- [Processing of Traffic Light Status Information in MPC-Planner](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/reports/10-Processing-of-Traffic-Light-Status-Information-in-MPC-Planner/2023-09_MA_Leong_Hubbertz/report.ipynb) by [Lingyue MA](https://github.com/ihrer73), [Yi Hong Leong](https://github.com/malao98) and [Michael Hubbertz](https://github.com/EcxaByte) (09/2023)

## Suggested Research Topics

All suggested research topics that have been presented in the video sequences are listed below. Note that the numbering has no special meaning and some numbers may be missing.

Before starting to work on a particular research project, take your time to check out the detailed task descriptions found in the repository. Clicking on the topic title in the table below will also directly take you to the detailed task description of each topic. You can also rewatch the task description video sequences (incl. [Overview](https://www.youtube.com/watch?v=2Krtx5EZ8fc) and [Outtro](https://www.youtube.com/watch?v=sUB7Ovy5At4)), see the links in the table below.

If you have an interesting research idea yourself that fits into the context of automated and connected driving, feel free to produce a research project report for your own topic. We would also be happy to integrate your project idea into this [acdc-research-projects](https://github.com/ika-rwth-aachen/acdc-research-projects) repository. Simply create a [pull request](https://docs.github.com/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests).

| No. | Topic | Task | Relevant ACDC Sections |
| :---: | --- | --- | --- |
| 01 | [Domain Adaptation for Semantic Image Segmentation](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/topics/01-Domain-Adaptation-for-Semantic-Image-Segmentation/task.ipynb) ([Video](https://www.youtube.com/watch?v=yBF92-fbf1E)) | Develop a methodology which optimizes a neural network for semantic image segmentation trained on public datasets with regard to its predictive performance on data affected by domain shift. | Image Segmentation | 
| 02 | [Domain Adaptation for Semantic Point Cloud Segmentation](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/topics/02-Domain-Adaptation-for-Semantic-Point-Cloud-Segmentation/task.ipynb) ([Video](https://www.youtube.com/watch?v=9JOQTWRy4Yw)) | Develop a methodology which optimizes a neural network for semantic point cloud segmentation trained on public datasets with regard to its predictive performance on data affected by domain shift. | Point Cloud Segmentation |
| 03 | [Domain Adaptation for Lidar Object Detection](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/topics/03-Domain-Adaptation-for-Lidar-Object-Detection/task.ipynb) ([Video](https://www.youtube.com/watch?v=_zUg3u6I-6Y)) | Develop a methodology which optimizes a neural network for lidar object detection trained on public datasets with regard to its predictive performance on data affected by domain shift. | Object Detection |
| 04 | [Time-Series Inverse Perspective Mapping](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/topics/04-Time-Series-Inverse-Perspective-Mapping/task.ipynb) ([Video](https://www.youtube.com/watch?v=j0lWEJSlWVM)) | Develop a methodology to fuse camera image information from multiple consecutive time steps in order to compute an advanced semantic grid map using the geometry-based Inverse Perspective Mapping (IPM) approach. | Image Segmentation, Camera-based Semantic Grid Mapping |
| 06 | [Cross-Modal Depth Estimation for Traffic Light Detection](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/topics/06-Cross-Modal-Depth-Estimation-for-Traffic-Light-Detection/task.ipynb) ([Video](https://www.youtube.com/watch?v=wQn04iutPaY)) | Develop a methodology for estimating the 3D position of a traffic light, given a binary image segmentation mask and a lidar point cloud. | Image Segmentation, Camera-based Semantic Grid Mapping |
| 10 | [Processing of Traffic Light Status Information in MPC-Planner](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/topics/10-Processing-of-Traffic-Light-Status-Information-in-MPC-Planner/task.ipynb) ([Video](https://www.youtube.com/watch?v=wd3zQYDuwrY)) | Identify and implement MPC-planner functionalities to improve trajectory planning at traffic lights. | Vehicle Guidance, Connected Driving |
| 11 | [Cloud-Based Neural Network Inference](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/topics/11-Cloud-Based-Neural-Network-Inference/task.ipynb) ([Video](https://www.youtube.com/watch?v=TebV9BErLVY)) | Implement and evaluate two different methodologies to moving neural network inference from automated vehicles to connected cloud servers. | Image Segmentation, Connected Driving |
| 12 | [Panoptic Image Segmentation](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/topics/12-Panoptic-Image-Segmentation/task.ipynb) | Develop a methodology to perform panoptic image segmentation and evaluate its performance on semantic segmentation, instance segmentation, and panoptic segmentation tasks. | Image Segmentation |

## Working on a Research Project

In order to start working on your selected research project, we recommend to fork this [acdc-research-projects](https://github.com/ika-rwth-aachen/acdc-research-projects) repository into your own GitHub account. This will allow you to not only locally work on a clone of the repository, but to also push your changes to your own GitHub. This way you can present your research project progress and outcome online and also have the ability to later post a pull request to the original [acdc-research-projects](https://github.com/ika-rwth-aachen/acdc-research-projects) repository. We would be happy to publish and endorse finished research projects.

More information about GitHub's fork and pull request concepts can be found here:
- [About Forks](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/about-forks)
- [About Pull Requests](https://docs.github.com/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests)

### 1. Forking the Repository

In order to fork this [acdc-research-projects](https://github.com/ika-rwth-aachen/acdc-research-projects) repository, click *Fork* in the top-right corner of the GitHub web interface. Confirm by clicking *Create fork* on the following dialog. You now have a copy of the original repository in your own GitHub account at `https://github.com/<YOUR_GITHUB_NAME>/acdc-research-projects`.

### 2. Cloning the Repository

You are now ready to clone your repository fork to your computer. When cloning the repository, make sure to recursively clone the required submodules. Additionally, [Git LFS](https://git-lfs.github.com/) is used to track large files such as images. Verify that Git LFS is installed before cloning. Note that you should not clone the original [acdc-research-projects](https://github.com/ika-rwth-aachen/acdc-research-projects) repository, but the forked repository that you have just created at `https://github.com/<YOUR_GITHUB_NAME>/acdc-research-projects`.

```bash
git lfs install
git clone --recurse-submodules https://github.com/<YOUR_GITHUB_NAME>/acdc-research-projects.git
```

### 3. Starting the JupyterLab Docker Container

As known from the [ACDC Jupyter Notebook Exercises](https://github.com/ika-rwth-aachen/acdc-notebooks), we suggest that you work on your research project from within our provided Docker container environment.

Once you have cloned the repository, enter the repository and launch the Docker container with the provided script:
```bash
# acdc-research-projects/ $
./docker/run.sh
```

This will start a JupyterLab server that you can access by opening the bottommost URL printed by the command output (starting with `http://127.0.0.1:8888/lab?token=`).

```
acdc-research-projects/ $ ./docker/run.sh 
Executing the command: jupyter lab

[ ... ]

[I 2023-01-06 15:02:05.081 ServerApp] Jupyter Server 1.10.2 is running at:
[I 2023-01-06 15:02:05.081 ServerApp] http://f62314f673dc:8888/lab?token=3342f3d444e484e6f1aa7b2ba056330b32b1cf4e62c4753f
[I 2023-01-06 15:02:05.081 ServerApp]  or http://127.0.0.1:8888/lab?token=3342f3d444e484e6f1aa7b2ba056330b32b1cf4e62c4753f
[I 2023-01-06 15:02:05.081 ServerApp] Use Control-C to stop this server and shut down all kernels (twice to skip confirmation).
[C 2023-01-06 15:02:05.083 ServerApp] 
    
    To access the server, open this file in a browser:
        file:///home/jovyan/.local/share/jupyter/runtime/jpserver-7-open.html
    Or copy and paste one of these URLs:
        http://f62314f673dc:8888/lab?token=3342f3d444e484e6f1aa7b2ba056330b32b1cf4e62c4753f
     or http://127.0.0.1:8888/lab?token=3342f3d444e484e6f1aa7b2ba056330b32b1cf4e62c4753f
```

### 4. Checking out the Project Task Description

In the JupyterLab web interface you will find all repository contents in the `acdc/` directory. Navigate to the topic directory of your selected research topic (`topics/<YOUR_SELECTED_TOPIC>`) using the explorer on the left side of the JupyterLab web interface. Double-click `task.ipynb` to open your detailed task description.

While you could have also viewed the task description in the GitHub web interface, you can now actually run the task description Jupyter Notebook as you are used from the [ACDC Jupyter Notebook Exercises](https://github.com/ika-rwth-aachen/acdc-notebooks). Note that not all task descriptions contain executable Python cells.

### 5. Checking out the Project Report Template

The final goal of any ACDC Research Project is to document your research in a well-written project report in the form of an executable Jupyter Notebook. This will allow other researchers to reproduce and build upon your work.

In the JupyterLab web interface, navigate to the template directory `template` and open the [project report template notebook `report.ipynb`](https://github.com/ika-rwth-aachen/acdc-research-projects/tree/main/template/report.ipynb). The report template contains a lot of information on how to structure a well-written project report, how to incorporate interactive code cells, how to install dependencies for reproducibility, and much more. Try to execute the code cells in the report template and take your time to understand how a final research project report could potentially look like.

### 6. Working on the Research Project

We recommend to only work on your project from within your selected topic's directory (`topics/<YOUR_SELECTED_TOPIC>`). Therefore, the first step is to copy the report template to your topic's directory. You can then get started to work on your research project by editing the copied report notebook in the JupyterLab web interface.

Note that a well-structured written research report is as important as developing and implementing the actual methodology. We therefore encourage you to keep your development efforts lean and also focus on producing a well-thought presentation of your research motive, methodology, and evaluation.

### 7. Publishing your Finished Research Project

As you work on your project, you should regularly `git commit` your progress in your forked repository. You can also `git push` your changes to your remote GitHub repository, such that the changes are publicly visible. Showcasing your progress and final research project report in your personal GitHub fork is a great way of demonstrating that you are capable of solving complex problems in the field of automated and connected driving. Once you have finished your project report, we also encourage you to create a [pull request](https://docs.github.com/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests) to this original [acdc-research-projects](https://github.com/ika-rwth-aachen/acdc-research-projects) repository. We would be very happy to then publish and endorse your research!
