/*
 * Parameters.cpp
 *
 *  Created on: Aug 19, 2021
 *      Author: jelavice
 */
#include "ground_plane_removal/Parameters.hpp"
#include <iomanip>

namespace ground_removal {

void loadParameters(const YAML::Node &node, GroundPlaneCropBoxParameters *p) {
	p->minX_ = node["crop_box_minX"].as<double>();
	p->maxX_ = node["crop_box_maxX"].as<double>();
	p->minY_ = node["crop_box_minY"].as<double>();
	p->maxY_ = node["crop_box_maxY"].as<double>();
	p->minZ_ = node["crop_box_minZ"].as<double>();
	p->maxZ_ = node["crop_box_maxZ"].as<double>();
}

std::ostream& operator<<(std::ostream& out, const GroundPlaneCropBoxParameters& p) {
  out << "┌────────────────────────────────────────────────────┐\n";
  out << "│                 GroundPlaneCropBoxParameters       │\n";
  out << "├─────────────────────────────────┬──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << " minX"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.minX_  << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "max X"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.minX_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "min Y"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.minY_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "max Y"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.maxY_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "min Z"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.minZ_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "max Z"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.maxZ_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";


  return out;
}

void loadParameters(const YAML::Node &node, GroundPlaneVoxelGridParameters *p) {
	p->leafSizeX_ = node["leaf_size_x"].as<double>();
	p->leafSizeY_ = node["leaf_size_y"].as<double>();
	p->leafSizeZ_ = node["leaf_size_z"].as<double>();
}

std::ostream& operator<<(std::ostream& out, const GroundPlaneVoxelGridParameters& p) {
  out << "┌────────────────────────────────────────────────────┐\n";
  out << "│                GroundPlaneVoxelGridParameters      │\n";
  out << "├─────────────────────────────────┬──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "size X"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.leafSizeX_  << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "size Y"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.leafSizeY_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";
  out << "│ " << std::left << std::setw(31) << "size Z"
      << " │ " << std::right << std::setw(16) << std::fixed
      << std::setprecision(4) << p.leafSizeZ_ << " │\n";
  out << "├─────────────────────────────────┼──────────────────┤\n";


  return out;
}

} // namespace ground_removal



