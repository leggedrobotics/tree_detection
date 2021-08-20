WORK IN PROGRESS. Cannot be compiled until a PR for grid_map is merged.

# tree_detection

## Overview

This package provides tree detection on pointcloud data. In particular, it is designed for detecting tree trunks which could later be used for tree grabbing and harvesting. It does not provide the full tree segmentation. The package was developed in the scope of [1].

The functionality was developed for an autonomous harvester use_case where the goal was to detect trees in a local map. Nonetheless, it can also be used on large maps to compute tree coordinates or get an estimate of tree density.

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

## Publications

If you are using packages from tree_detection, please consider adding the following citation to your publication:

[1] Jelavic, E., Jud, D., Egli, P. and Hutter, M., 2021. Towards Autonomous Robotic Precision Harvesting. arXiv preprint arXiv:2104.10110.
 
    @article{jelavic2021towards,  
       title   = {Towards Autonomous Robotic Precision Harvesting},  
       author  = {Jelavic, Edo and Jud, Dominic and Egli, Pascal and Hutter, Marco},  
       journal = {arXiv preprint arXiv:2104.10110},  
       year    = {2021}
    }
   


