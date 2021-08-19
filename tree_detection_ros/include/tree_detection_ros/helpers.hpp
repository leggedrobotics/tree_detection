/*
 * helpers.hpp
 *
 *  Created on: Aug 18, 2021
 *      Author: jelavice
 */

#pragma once
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <Eigen/Core>
#include "tree_detection/common.hpp"
#include "tree_detection/typedefs.hpp"
#include "sensor_msgs/PointCloud2.h"
#include "ground_plane_removal/Parameters.hpp"
#include <chrono>

namespace tree_detection {

class Color: public std_msgs::ColorRGBA {
public:
	Color();
	Color(double red, double green, double blue);
	Color(double red, double green, double blue, double alpha);

	static const Color White() {
		return Color(1.0, 1.0, 1.0);
	}
	static const Color Black() {
		return Color(0.0, 0.0, 0.0);
	}
	static const Color Gray() {
		return Color(0.5, 0.5, 0.5);
	}
	static const Color Red() {
		return Color(1.0, 0.0, 0.0);
	}
	static const Color Green() {
		return Color(0.0, 1.0, 0.0);
	}
	static const Color Blue() {
		return Color(0.0, 0.0, 1.0);
	}
	static const Color Yellow() {
		return Color(1.0, 1.0, 0.0);
	}
	static const Color Orange() {
		return Color(1.0, 0.5, 0.0);
	}
	static const Color Purple() {
		return Color(0.5, 0.0, 1.0);
	}
	static const Color Chartreuse() {
		return Color(0.5, 1.0, 0.0);
	}
	static const Color Teal() {
		return Color(0.0, 1.0, 1.0);
	}
	static const Color Pink() {
		return Color(1.0, 0.0, 0.5);
	}
	static const Color Magenta() {
		return Color(0.78, 0.0, 0.9);
	}
};

visualization_msgs::Marker getTreeCylinderMarker(const Eigen::Vector3d &treeCoordinates,
		const ClusterDimensions &clusterDimension, int markerId, const ros::Time &timestamp, const std::string &frameId,
		const Color &color);

jsk_recognition_msgs::BoundingBox getTreeBBMarker(const Eigen::Vector3d &treeCoordinates,
		const ClusterDimensions &clusterDimension, int markerId, const ros::Time &timestamp,
		const std::string &frameId);

PointCloud::Ptr fromRos(const sensor_msgs::PointCloud2 &cloud);
sensor_msgs::PointCloud2 toRos(const PointCloud &cloud);
PointCloud::Ptr loadPointcloudFromPcd(const std::string &filename);

inline void printTimeElapsed(const std::chrono::steady_clock::time_point& start, const std::string& prefix) {
  const auto stop = std::chrono::steady_clock::now();
  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count() / 1e6;
  std::cout << prefix << duration << " sec \n";
}

} // namespace tree_detection
