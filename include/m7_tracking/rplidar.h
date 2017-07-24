#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/PoseArray.h>
#include <vector>
#include <array>
#include <string.h>
#include <stdio.h>
#include <sensor_msgs/LaserScan.h>

#define FREQUENCY 6.25
#define DEFAULT_SCAN_TOPIC "laser/scan"
#define DEFAULT_ODOM_FRAME_NAME "odom"
#define DEFAULT_COM_FRAME_NAME "base_link"
#define DEFAULT_WORLD_FRAME_NAME "world"
#define DEFAULT_LIDAR_FRAME_NAME "lidar"
#define OBSTACLE_SPACE_THRESHOLD 1.5 //0.35 //0.17. 0.34 is the diameter of a roomba

class ObstacleDetector{
public:

	ObstacleDetector(void);
	void lidarCallback(const sensor_msgs::LaserScanConstPtr& input);
	void run();
	void removeDuplicates(std::vector<tf::Vector3>& worldLidarPoses);
	geometry_msgs::PoseArray getPoses()
	{
		return obstaclePoses;
	}

	std::vector<tf::Vector3> getPoseVector()
	{
		std::vector<tf::Vector3> result;
		for(auto e: obstaclePoses.poses)
		{
			result.push_back(tf::Vector3(e.position.x, e.position.y, e.position.z));
		}

		return result;
	}

private:

	ros::NodeHandle nh;
	ros::Subscriber lidarSub;
//	ros::Publisher obstaclePublisher;
	sensor_msgs::LaserScan lidarInput;
	std_msgs::Header inpHeader;
	std::string lidar_frame;
	std::string world_frame;
	geometry_msgs::PoseArray obstaclePoses;
};
