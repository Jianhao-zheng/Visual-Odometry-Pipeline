# Monocular Visual Odometry Pipeline

> Mini project for [*Vision Algorithms for Mobile Robots*](http://rpg.ifi.uzh.ch/teaching.html) given by [Prof. Davide Scaramuzza](http://rpg.ifi.uzh.ch/people_scaramuzza.html), 2021
>
> Implementation of a working, simple, monocular visual odometry (VO) pipeline with the following implementation details:
>
> - KLT-based or descriptor matching for bootstrapping initialization
> - KLT Tracking of feature points across frames following by RANSAC
> - Triangulation of new landmarks
> - Local pose refinement through optimization
> - Release two [custom sequences](#custom-datasets)



## Authors

- **Jianhao ZHENG**
- **Yujie HE**



## Data

### Provided datasets

Download data from [RPG VAME course website](http://rpg.ifi.uzh.ch/teaching.html) and place them in the following structure

```shell
├── data
│   ├── kitti
│   └── malaga
│   └── parking
```

### Custom datasets

> For more details, you could refer to readme in following subfolder

- [`epfl_parking`](https://github.com/Jianhao-zheng/Visual-Odometry-Pipeline/tree/master/data/epfl_parking)
- [`lausanne_center_nav`](https://github.com/Jianhao-zheng/Visual-Odometry-Pipeline/tree/master/data/lausanne_center_nav)



## Demo

| Test sequences      | Demo                                                         | Video                                                        |
| ------------------- | ----------------------------------------------- | ------------------------------------------------------------ |
| KITTI seq05         | ![epfl_parking](./gifs/kitti.gif) | [[link](https://www.youtube.com/watch?v=ByywzaIwTSM&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=1)] |
| malaga              | ![epfl_parking](./gifs/malaga.gif) | [[link](https://www.youtube.com/watch?v=l-Jklm77tNg&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=2)] |
| parking             | ![epfl_parking](./gifs/parking.gif) | [[link](https://www.youtube.com/watch?v=Xut0iuFSy8o&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=3)] |
| epfl_parking        | ![epfl_parking](./gifs/epfl_parking.gif) | [[link](https://www.youtube.com/watch?v=eWNpX07L4_A&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=4)] |
| lausanne_center_nav | ![epfl_parking](./gifs/lausanne_center_nav.gif) | [[link](https://www.youtube.com/watch?v=qSgeN7ElPik&list=PLisWEer2ynw1Ws1_km6y-xXDAIyvJ9weM&index=5)] |

## Scripts

| **Test passed**                                              |
| ------------------------------------------------------------ |
| [![matlab-2021b](https://img.shields.io/badge/matlab-2021b-yellow.svg)](https://www.mathworks.com/products/matlab.html) [![matlab-2020b](https://img.shields.io/badge/matlab-2020b-blue.svg)](https://www.mathworks.com/products/matlab.html) with Computer Vision Toolbox, Image Processing Toolbox, Optimization Toolbox |

```plaintext
```

