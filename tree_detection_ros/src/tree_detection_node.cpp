#include "tree_detection_ros/TreeDetectorRos.hpp"
#include "tree_detection_ros/helpers.hpp"
#include "tree_detection_ros/creators.hpp"


int main(int argc, char **argv) {
	using namespace tree_detection;
	ros::init(argc, argv, "tree_detection_node");
	ros::NodeHandlePtr nh(new ros::NodeHandle("~"));
	const std::string pcdFilename = nh->param<std::string>("pcd_filepath", "");
	std::cout << "Loading pointcloud from: " << pcdFilename << std::endl;
	PointCloud::Ptr loadedCloud = loadPointcloudFromPcd(pcdFilename);
	const std::string frameId = "map";
	loadedCloud->header.frame_id = frameId;
	ros::Publisher cloudPub = nh->advertise<sensor_msgs::PointCloud2>("input_cloud", 1, true);
	sensor_msgs::PointCloud2 cloudMsg = toRos(*loadedCloud);
	cloudMsg.header.frame_id = frameId;
	cloudMsg.header.stamp = ros::Time::now();
	cloudPub.publish(cloudMsg);

	const std::string configFilePath = nh->param<std::string>("config_filepath", "");
	std::cout << "Loading parameters from: " << configFilePath << std::endl;

	//here create all the shizzle
	TreeDetectorRos treeDetector(nh);
	TreeDetectionParameters treeDetectParam;
	loadParameters(configFilePath, &treeDetectParam);
	treeDetector.setTreeDetectionParameters(treeDetectParam);
	const std::string groundRemovalStrategy = nh->param<std::string>("ground_removal_strategy", "");
	std::shared_ptr<ground_removal::GroundPlaneRemover> groundRemover = ground_removal::groundRemoverFactory(groundRemovalStrategy,
			configFilePath,nh);


	//here crop the ground plane
	const auto startTime = std::chrono::steady_clock::now();
	groundRemover->setInputCloud(*loadedCloud);
	groundRemover->removeGroundPlane();
	printTimeElapsed(startTime,"Time elapsed for ground removal: ");

	//here detect the trees
	PointCloud::ConstPtr noGroundPlaneCloud = groundRemover->getCloudWithoutGroundPlanePtr();
	treeDetector.setInputPointCloud(*noGroundPlaneCloud);
	treeDetector.setCloudFrameId(frameId);
	treeDetector.setCloudTimestamp(ros::Time::now());
	treeDetector.detectTrees();
	printTimeElapsed(startTime,"Total time: ");


	ros::Publisher noGroundCloudPub = nh->advertise<sensor_msgs::PointCloud2>("no_ground_cloud", 1, true);
	sensor_msgs::PointCloud2 noGroundCloudMsg = toRos(*noGroundPlaneCloud);
	noGroundCloudMsg.header.frame_id = frameId;
	noGroundCloudMsg.header.stamp = ros::Time::now();
	noGroundCloudPub.publish(noGroundCloudMsg);

	ros::spin();

	return 0;
}
