/*
 * creators.hpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */
#pragma once

#include "ground_plane_removal/GroundPlaneRemover.hpp"
#include "ground_plane_removal/ElevationMapGroundPlaneRemover.hpp"
#include "ground_plane_removal/Parameters.hpp"


namespace ground_removal {

void loadParameters(const std::string &filename, ground_removal::GroundPlaneRemoverParam *p);
void loadParameters(const std::string &filename, ground_removal::ElevationMapGroundPlaneRemoverParam *p);
std::unique_ptr<ground_removal::GroundPlaneRemover> groundRemoverFactory(const std::string &strategy,
		const std::string &configFile, ros::NodeHandlePtr nh);

} // namespace ground_removal
