WORK IN PROGRESS. Cannot be compiled until a PR for grid_map is merged.

# tree_detection

## Overview

This package provides tree detection on pointcloud data. In particular, it is designed for detecting tree trunks which could later be used for tree grabbing and harvesting. It does not provide the full tree segmentation. The package was developed in the scope of [1].

The functionality was developed for an autonomous harvester use_case where the goal was to detect trees in a local map. Nonetheless, it can also be used on large maps to compute tree coordinates or get an estimate of tree density.

Released under [BSD 3-Clause license](LICENSE).

**Author:** Edo Jelavic

**Maintainer:** Edo Jelavic, [jelavice@ethz.ch](jelavice@ethz.ch)


## Installation


## Publications

If you are using packages from tree_detection, please consider adding the following citation to your publication:

[1] Jelavic, E., Jud, D., Egli, P. and Hutter, M., 2021. Towards Autonomous Robotic Precision Harvesting. arXiv preprint arXiv:2104.10110.
 
    @article{jelavic2021towards,  
       title   = {Towards Autonomous Robotic Precision Harvesting},  
       author  = {Jelavic, Edo and Jud, Dominic and Egli, Pascal and Hutter, Marco},  
       journal = {arXiv preprint arXiv:2104.10110},  
       year    = {2021}
    }
   


