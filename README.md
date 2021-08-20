WORK IN PROGRESS. Cannot be compiled until a PR for grid_map is merged.

# tree_detection

## Overview

This package provides tree detection on pointcloud data. In particular, it is designed for detecting tree trunks which could later be used for tree grabbing and harvesting. It does not provide the full tree segmentation with the crown. The package was developed in the scope of [1] (see below).

The functionality was developed for an autonomous harvester use case where the goal is to detect trees in a local map. Nonetheless, it can also be used on large maps to compute tree coordinates or get an estimate of tree density.

Released under [BSD 3-Clause license](LICENSE).

**Author:** Edo Jelavic

**Maintainer:** Edo Jelavic, [jelavice@ethz.ch](jelavice@ethz.ch)


## Packages in this repo
This repository consists of following packages:

* ***ground_plane_removal*** is a lightweight package for ground plane removal. 
* ***tree_detection*** implements the core algorithm for tree detection. 
* ***tree_detection_ros*** adds visualization and I/O handling for easier usage. This package is tightly integrated with ROS.


## Installation

Clone the following dependencies:
```bash
# in your source folder `src`
git clone https://github.com/ANYbotics/grid_map.git
```

Install the ROS and library dependencies with:
```bash
sudo apt install -y ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-jsk-recognition-msgs ros-noetic-tf2-geometry 
# OR, use rosdep in your source folder `src` 
sudo rosdep install -yr --from-paths .
```
Recommended to build in release mode for performance (`catkin config -DCMAKE_BUILD_TYPE=Release`)

Build with:  
```bash
catkin build tree_detection_ros
```

## Usage

The example datasets can be downloaded [here](https://drive.google.com/drive/folders/1m_sRtMN5n6-ShQvnbCfedKIVpoupao5u?usp=sharing). The folder contains three forest patches, one without clutter (forest3.pcd) and two with a lot of branch clutter (forest1.pcd and forest2.pcd). 

Modify the `pcd_filepath` inside the `tree_detection.launch` to point the location where the point clouds `.pcd` files are stored.

Run with:
```bash
roslaunch tree_detection_ros tree_detection.launch
```

The node publishes the input pointcloud, the pointcloud with the ground plane removed and the elevation map used to remove the ground plane. 




## Publications

If you are using packages from tree_detection, please consider adding the following citation to your publication:

[1] Jelavic, E., Jud, D., Egli, P. and Hutter, M., 2021. Towards Autonomous Robotic Precision Harvesting. arXiv preprint arXiv:2104.10110.
 
    @article{jelavic2021towards,  
       title   = {Towards Autonomous Robotic Precision Harvesting},  
       author  = {Jelavic, Edo and Jud, Dominic and Egli, Pascal and Hutter, Marco},  
       journal = {arXiv preprint arXiv:2104.10110},  
       year    = {2021}
    }
   


