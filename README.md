- [**Overview**](#overview)
- [**Codebase**](#codebase)
  - [Machine specifications](#machine-specifications)
  - [Dev Environment](#dev-environment)
  - [How to run](#how-to-run)
  - [Folder Structure](#folder-structure)
- [**Demo**](#demo)
- [**Data**](#data)
  - [Provided datasets](#provided-datasets)
  - [Customized datasets](#customized-datasets)
  - [Related repos](#related-repos)



# Monocular Visual Odometry Pipeline

> Authors: Jianhao Zheng, Yujie He

## Overview

Mini project for [*Vision Algorithms for Mobile Robots*](http://rpg.ifi.uzh.ch/teaching.html) given by [Prof. Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html), 2021

Implementation of a working, simple, monocular visual odometry (VO) pipeline with the following implementation details:

- KLT-based or descriptor matching for bootstrapping initialization
- KLT Tracking of feature points across frames following by RANSAC
- Triangulation of new landmarks
- Local pose refinement through optimization
- Bundle adjustment for better pose estimation
- Release two [custom sequences](#custom-datasets)


## Codebase

### Machine specifications

- CPU: AMD Ryzen 7 5800H, 3.2 GHz, 16 logical process
- RAM: 16GB

### Dev Environment

| **Test passed**                                              |
| ------------------------------------------------------------ |
| [![matlab-2021b](https://img.shields.io/badge/matlab-2021b-yellow.svg)](https://www.mathworks.com/products/matlab.html) [![matlab-2020b](https://img.shields.io/badge/matlab-2020b-blue.svg)](https://www.mathworks.com/products/matlab.html) |

- Toolbox used
  - Computer Vision Toolbox
  - Image Processing Toolbox
  - Optimization Toolbox

### How to run
Download dataset and copy them to the right folder. For details on setting data, please refer to [**Data**](#data). To test the VO pipeline without bundle adjustment, run `main_demo.m`. Change variable `ds` to switch the testing dataset. 

For VO with bundle adjustment, plese run `main_BA.m` and make sure `hyper_paras.is_BA` is `true`. (For now, only tested in parking dataset, ds = 2)

### Folder Structure

```
Visual-Odometry-Pipeline/
├── Continuous_operation # (matlab) implemented algorithms about continuous operation
├── Initialization # (matlab) implemented algorithms about initialization
├── utils # (matlab) utility function for data processing and visualization in the pipeline
├── eval_notebook # (python) scripts to evaluate performance between different methods
├── main_BA.m # (matlab) script to demonstrate implemented method with bundle adjustment on `parking` data
├── main_demo.m # (matlab) script to demonstrate implemented method without bundle adjustment for every dataset
├── main_eval.m # (matlab) script to batch evaluate the implemented method with different features on `KITTI seq05` data
├── data # 3 data sequences provided by VAME team and 2 customized sequences
├── gifs # demonstration gifs
├── README.md
├── ...
```


## Demo

| Test sequences      | Demo                                                         | Video                                                        |
| ------------------- | ----------------------------------------------- | ------------------------------------------------------------ |
| KITTI seq05         | ![epfl_parking](./gifs/kitti.gif) | [[link](https://www.youtube.com/watch?v=ByywzaIwTSM&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=1)] |
| malaga              | ![epfl_parking](./gifs/malaga.gif) | [[link](https://www.youtube.com/watch?v=l-Jklm77tNg&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=2)] |
| parking             | ![epfl_parking](./gifs/parking.gif) | [[link](https://www.youtube.com/watch?v=Xut0iuFSy8o&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=3)] |
| epfl_parking        | ![epfl_parking](./gifs/epfl_parking.gif) | [[link](https://www.youtube.com/watch?v=eWNpX07L4_A&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=4)] |
| lausanne_center_nav | ![epfl_parking](./gifs/lausanne_center_nav.gif) | [[link](https://www.youtube.com/watch?v=qSgeN7ElPik&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=5)] |


## Data

### Provided datasets

Download data from [RPG VAME course website](http://rpg.ifi.uzh.ch/teaching.html) and place them in the following structure

```shell
├── data
│   ├── kitti
│   └── malaga
│   └── parking
```

### Customized datasets

> For more details, you could refer to readme in following subfolder

- [`epfl_parking`](https://github.com/Jianhao-zheng/Visual-Odometry-Pipeline/tree/master/data/epfl_parking)
- [`lausanne_center_nav`](https://github.com/Jianhao-zheng/Visual-Odometry-Pipeline/tree/master/data/lausanne_center_nav)



### Related repos

- [hibetterheyj](https://github.com/hibetterheyj)/**[VideoIMUCapture-Android](https://github.com/hibetterheyj/VideoIMUCapture-Android)** for camera calibration and image preprocessing (undistortion & resize)

