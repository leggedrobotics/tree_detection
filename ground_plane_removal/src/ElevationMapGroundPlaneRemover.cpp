/*
 * ElevationMapGroundPlaneRemover.cpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */

#include "ground_plane_removal/ElevationMapGroundPlaneRemover.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/GridMap.h"

#ifdef GROUND_PLANE_REMOVAL_OPENMP_FOUND
#include <omp.h>
#endif

namespace ground_removal {

namespace {
const std::string elevationLayer = "elevation";
ros::Publisher gridMapPub;

template<typename T>
T clamp(const T val, const T lower, const T upper) {
	return std::max(lower, std::min(val, upper));
}

void bindToRange(const grid_map::Matrix &data, grid_map::Index *id) {
	const int i = clamp(id->x(), 0, (int) data.rows());
	const int j = clamp(id->y(), 0, (int) data.cols());
	id->x() = i;
	id->y() = j;
}

void computePointsForLocalTerrainFit(double R, const grid_map::GridMap &gridMap, std::vector<double> *xs,
		std::vector<double> *ys, int downSamplingFactor = 1) {
	downSamplingFactor = downSamplingFactor > 0 ? downSamplingFactor : 1; // safety check
	const int minNumPoints = 8; //todo magic
	xs->clear();
	ys->clear();
	const double res = gridMap.getResolution();
	const int approxNumCellsInCircle = std::ceil(R * R * M_PI / (res * res) * 1.2 / downSamplingFactor);
	xs->reserve(approxNumCellsInCircle);
	ys->reserve(approxNumCellsInCircle);
	int i = 0;
	for (grid_map::SpiralIterator it(gridMap, grid_map::Position(0.0, 0.0), R); !it.isPastEnd(); ++it, ++i) {
		if (i > minNumPoints && !(i % downSamplingFactor == 0)) { //ensure minimal minNumPoints points
			continue;
		}
		const grid_map::Index circleCellId(*it);
		grid_map::Position circleCellPos;
		gridMap.getPosition(circleCellId, circleCellPos);
		xs->push_back(circleCellPos.x());
		ys->push_back(circleCellPos.y());
	}
	//ensure that the center is always in
	xs->push_back(0.0);
	ys->push_back(0.0);
}

void medianFiltering(grid_map::GridMap *mapPtr, double estimationRadius, int downsampleFactorNumData, int nThreads) {

	auto &map = *mapPtr;
	const grid_map::Matrix &gridMapData = mapPtr->get(elevationLayer);
	grid_map::Matrix gridMapDataCopy = mapPtr->get(elevationLayer);
	unsigned int linearGridMapSize = mapPtr->getSize().prod();
	std::vector<double> xs, ys;
	computePointsForLocalTerrainFit(estimationRadius, map, &xs, &ys, downsampleFactorNumData);
	const int maxNpoints = xs.size();

#ifdef GROUND_PLANE_REMOVAL_OPENMP_FOUND
	omp_set_num_threads(nThreads);
#pragma omp parallel for schedule(static)
#endif
	for (unsigned int linearIndex = 0; linearIndex < linearGridMapSize; ++linearIndex) {
		std::vector<double> goodHs;
		goodHs.reserve(maxNpoints);
		const grid_map::Index currentCellId(grid_map::getIndexFromLinearIndex(linearIndex, mapPtr->getSize()));
		grid_map::Position currentCellPosition;
		map.getPosition(currentCellId, currentCellPosition);
		// fill data
		for (int i = 0; i < maxNpoints; ++i) {
			grid_map::Position currentPointPos(currentCellPosition.x() + xs[i], currentCellPosition.y() + ys[i]);
			grid_map::Index currentPointId;
			map.getIndex(currentPointPos, currentPointId);
			bindToRange(gridMapData, &currentPointId);
			const double h = gridMapData(currentPointId.x(), currentPointId.y());
			bool isAddPoint = std::isfinite(h);
			if (isAddPoint) {
				goodHs.push_back(h);
			}
		}
		std::nth_element(goodHs.begin(), goodHs.begin() + goodHs.size() / 2, goodHs.end());
		gridMapDataCopy(currentCellId.x(), currentCellId.y()) = goodHs[goodHs.size() / 2];
	}  // end iterating over linear index
	map.get(elevationLayer) = gridMapDataCopy;
}

} // namespace

ElevationMapGroundPlaneRemover::ElevationMapGroundPlaneRemover(ros::NodeHandle *nh) :
		nh_(nh) {
}

void ElevationMapGroundPlaneRemover::setParameters(const GroundPlaneRemoverParam &p) {
	auto param = static_cast<const ElevationMapGroundPlaneRemoverParam*>(&p);
	param_ = *param;
	pclToGridMap_.setParameters(param_.pclConverter_.get());
}
const ElevationMapGroundPlaneRemoverParam& ElevationMapGroundPlaneRemover::getParameters() const {
	return param_;
}

const grid_map::GridMap& ElevationMapGroundPlaneRemover::getElevationMap() const {
	return elevationMap_;
}

void ElevationMapGroundPlaneRemover::removeGroundPlane() {
	pclToGridMap_.setInputCloud(inputCloud_);
	pclToGridMap_.initializeGridMapGeometryFromInputCloud();
	pclToGridMap_.addLayerFromInputCloud(elevationLayer);
	auto gridMap = pclToGridMap_.getGridMap();
	if (param_.isUseMedianFiltering_) {
		medianFiltering(&gridMap, param_.medianFilteringRadius_, param_.medianFilterDownsampleFactor_,
				param_.pclConverter_.get().numThreads_);
	}

	elevationMap_ = gridMap;
	noGroundPlaneCloud_.reset(new PointCloud);
	noGroundPlaneCloud_->points.reserve(filterCloud_->size());
//	const auto &data = gridMap.get(elevationLayer);
	int numPts = 0;
	for (int i = 0; i < filterCloud_->size(); ++i) {
		double x = filterCloud_->points[i].x;
		double y = filterCloud_->points[i].y;
		snapToMapLimits(gridMap, &x, &y);
		const double z = filterCloud_->points[i].z;
		const double h = gridMap.atPosition(elevationLayer, grid_map::Position(x, y));
		const bool isSkip = (z < h + param_.minHeightAboveGround_) || ( z > h + param_.maxHeightAboveGround_ );
		if (isSkip) {
			continue;
		}
		noGroundPlaneCloud_->points.push_back(filterCloud_->points[i]);
		numPts++;
	}
	noGroundPlaneCloud_->points.resize(numPts);
	noGroundPlaneCloud_->width = noGroundPlaneCloud_->points.size();
	noGroundPlaneCloud_->height = 1;
	noGroundPlaneCloud_->is_dense = true;

	if (nh_ != nullptr) {
		gridMapPub = nh_->advertise<grid_map_msgs::GridMap>("elevation_map", 1, true);
		grid_map_msgs::GridMap msg;
		grid_map::GridMapRosConverter::toMessage(gridMap, msg);
		msg.info.header.frame_id = inputCloud_->header.frame_id;
		gridMapPub.publish(msg);
	}
}

void ElevationMapGroundPlaneRemover::snapToMapLimits(const grid_map::GridMap &gm, double *x, double *y) const {
	if (gm.isInside(grid_map::Position(*x, *y))) {
		return;
	}
	const auto snapped = gm.getClosestPositionInMap(grid_map::Position(*x, *y));
	const auto p0 = gm.getPosition();
	// todo raise an issue on grid_map github about this

	const double hackingFactor = 0.99;
	*x = (snapped.x() - p0.x()) * hackingFactor + p0.x();
	*y = (snapped.y() - p0.y()) * hackingFactor + p0.y();
}

void ElevationMapGroundPlaneRemover::setFilterCloudPtr(PointCloud::ConstPtr filterCloud) {
  setFilterCloud(*filterCloud);
}

void ElevationMapGroundPlaneRemover::setFilterCloud(const PointCloud& filterCloud) {
  filterCloud_.reset(new PointCloud);
  *filterCloud_ = filterCloud;
}

}  // namespace ground_removal
