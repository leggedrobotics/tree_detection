WORK IN PROGRESS. Cannot be compiled until a PR for grid_map is merged.

# tree_detection

## Overview

This package provides tree detection on pointcloud data. In particular, it is designed for detecting tree trunks which could later be used for tree grabbing and harvesting. It does not provide the full tree segmentation with the crown. The package was developed in the scope of [1] (see below).

The functionality was developed for an autonomous harvester where the goal is to detect trees in a local map. Nonetheless, it can also be used on large maps to compute tree coordinates or get an estimate of tree density.

Released under [BSD 3-Clause license](LICENSE).

**Author:** Edo Jelavic

**Maintainer:** Edo Jelavic, [jelavice@ethz.ch](jelavice@ethz.ch)


## Packages in this repo
This repository consists of following packages:

* ***ground_plane_removal*** is a lightweight package for ground plane removal. 
* ***tree_detection*** implements the core algorithm for tree detection. 
* ***tree_detection_ros*** adds visualization and I/O handling for easier usage. This package is tightly integrated with ROS.


## Algorithm description

The algorithm first attempts to remove the gound plane. This can be done using two strategies, either a simple box filter (for which the user needs to provide the cropping region) or using an elevation map. In case the elevation map strategy is used, the algorithm first computes the elevation map and then uses it to remove the ground plane. The elevaiton map strategy is implemented in a different package (see the link [here](https://github.com/ANYbotics/grid_map/tree/master/grid_map_pcl)). The elevation map extraction algorithm is described in more detail in [1]. Before proceeding with the pointcloud processing the median filter is applied to the elevation map.

Once the ground plane is removed, the algorithm proceeds with Euclidean cluster extraction and performs some checks on those clusters to make sure that the extracted cluster is indeed a tree. The more detailed description can be found in [1].

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

This package can also work with forests on inclined surfaces. Two examples are provided with dense maps (forest4.pcd and forest5.pcd). Note that for these datasets, we recoomend to use a slightly different tuning. See `tree_detection_dataset_4_and_5.yaml`. 

Modify the `pcd_filepath` inside the `tree_detection.launch` to point the location where the point clouds `.pcd` files are stored.

Run with:
```bash
roslaunch tree_detection_ros tree_detection.launch
```
The node publishes the input pointcloud, the pointcloud with the ground plane removed and the elevation map used to remove the ground plane. The detected trees will be marked with green cylinders and bounding boxes. The visualization shoud appar in the Rviz window.

Note that the computation can last for up to 7-9 minutes for larger datasets (e.g. forest4.pcd and forest5.pcd). This can be shortened if you downsample or voxelize the pointcloud.


## Parameters

* `ground_plane_removal/cropbox` - Limits (XYZ) for the ground plane removal. The points contained within the box specified by the user will be removed and the tree detection will proceed on the remaining pointcloud.
* `ground_plane_removal/elevation_map` - Parameters for elevation map extraction from the pointcloud are described [here](https://github.com/ANYbotics/grid_map/tree/master/grid_map_pcl). We add a couple of new parameters which are listed below.
* `ground_plane_removal/min_height_above_ground` - minimal height above the extracted elevation map to keep the points. Any points below this threshold will be discarded.
* `ground_plane_removal/max_height_above_ground` - maximal height above the elevation map to keep the points. Any points above this threshold will be discarded.
* `ground_plane_removal/is_use_median_filter` - Whether to apply the median filter on the elevation map.
* `ground_plane_removal/median_filtering_radius` - Median filtering radius for the elevation map. For wach cell, the filter will take the data from the neighbouring cells withing specified radius and apply the median filter. The number of neighbouring points depends on the cell size and this parameter.
* `ground_plane_removal/median_filter_points_downsample_factor` - If one wishes to have sparser coverage, you can apply downsampling. The algorithm will calculate all the points within the specified radius and divide their number by this factor. E.g., (if there are 50 neighbouring points within the radius and the downsample factor is set to 5, 10 points will be used for median filtering).

## Publications

If you are using packages from tree_detection, please consider adding the following citation to your publication:

[1] Jelavic, E., Jud, D., Egli, P. and Hutter, M., 2021. Towards Autonomous Robotic Precision Harvesting. arXiv preprint arXiv:2104.10110.
 
    @article{jelavic2021towards,  
       title   = {Towards Autonomous Robotic Precision Harvesting},  
       author  = {Jelavic, Edo and Jud, Dominic and Egli, Pascal and Hutter, Marco},  
       journal = {arXiv preprint arXiv:2104.10110},  
       year    = {2021}
    }
   


