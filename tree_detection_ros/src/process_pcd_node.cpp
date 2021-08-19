/*
 * downsample_pcd.cpp
 *
 *  Created on: Mar 15, 2021
 *      Author: jelavice
 */

#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>

using Point = ::pcl::PointXYZ;
using Pointcloud = ::pcl::PointCloud<Point>;
const std::string fixedFrame = "map";
Pointcloud::Ptr workingCloud;

Eigen::Vector3d minBound, maxBound;
bool isApplyCropBox = false;

bool isApplyRandomDownsample = true;
unsigned int desiredNumPoints = 1000000;

void loadParameters(const ros::NodeHandle &nh)
{
  minBound.x() = nh.param<double>("cropBox/minX", 0.0);
  minBound.y() = nh.param<double>("cropBox/minY", 0.0);
  minBound.z() = nh.param<double>("cropBox/minZ", 0.0);
  maxBound.x() = nh.param<double>("cropBox/maxX", 0.0);
  maxBound.y() = nh.param<double>("cropBox/maxY", 0.0);
  maxBound.z() = nh.param<double>("cropBox/maxZ", 0.0);
  isApplyCropBox = nh.param<bool>("cropBox/is_apply", false);

  //downsample
  isApplyRandomDownsample = nh.param<bool>("random_downsample/is_apply", false);
  desiredNumPoints = nh.param<int>("random_downsample/desired_n_points", 1000000);
}

Pointcloud::Ptr loadPointcloudFromPcd(const std::string& filename)
{
  Pointcloud::Ptr cloud(new Pointcloud);
//  pcl::PCLPointCloud2 cloudBlob;
//  pcl::io::loadPCDFile(filename, cloudBlob);
//  pcl::fromPCLPointCloud2(cloudBlob, *cloud);
  pcl::io::loadPCDFile(filename, *cloud);
  return cloud;
}

void savePointcloud(const std::string& filename, Pointcloud::ConstPtr cloud)
{
//  pcl::io::savePCDFileASCII (filename, *cloud);
  pcl::io::savePCDFileBinary(filename, *cloud);
}

void cropPointCloud(Pointcloud::Ptr inputCloud)
{

  if (!isApplyCropBox){
    return;
  }

  std::cout << "Applying cropbox filter with min bound: " << minBound.transpose();
  std::cout << " and max bound: " << maxBound.transpose() << "\n";
  pcl::CropBox<Point> boxFilter;
  boxFilter.setMin(Eigen::Vector4f(minBound.x(), minBound.y(), minBound.z(), 1.0));
  boxFilter.setMax(Eigen::Vector4f(maxBound.x(), maxBound.y(), maxBound.z(), 1.0));
  boxFilter.setInputCloud(inputCloud);
  boxFilter.filter(*inputCloud);

}

void downsamplePointCloud(Pointcloud::Ptr inputCloud)
{
  if (inputCloud->size() <= desiredNumPoints || !isApplyRandomDownsample) {
    return;
  }

  std::cout << "Input cloud has: " << inputCloud->size() << " points, downsampling to: "
            << desiredNumPoints << std::endl;
  pcl::RandomSample<Point> random;
  random.setSample(desiredNumPoints);
  random.setInputCloud(inputCloud);
  random.filter(*inputCloud);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "process_pcd_node");
  ros::NodeHandle nh("~");
  const auto start = ros::Time::now();
  const std::string pclFilename = nh.param<std::string>("pcd_filename", "");
  loadParameters(nh);
  std::cout << "Loading cloud: " << pclFilename << std::endl;
  workingCloud = loadPointcloudFromPcd(pclFilename);
  std::cout << "Cloud size: " << workingCloud->size() << " points \n";
  cropPointCloud(workingCloud);
  downsamplePointCloud(workingCloud);

  const std::string outFilename = nh.param<std::string>("pcd_file_path_out", "");
  std::cout << "Saving cloud size of: " << workingCloud->size() << std::endl;
  savePointcloud(outFilename, workingCloud);
  const auto end = ros::Time::now();
  std::cout << "succesfully finished" << std::endl;
  std::cout << "duration: " << (end - start).toSec() << " sec \n";
  return 0;
}

