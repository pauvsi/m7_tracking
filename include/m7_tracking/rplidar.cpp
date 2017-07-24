#include "rplidar.h"



ObstacleDetector::ObstacleDetector(void)
{
		this->lidar_frame = DEFAULT_LIDAR_FRAME_NAME;
		this->world_frame = DEFAULT_WORLD_FRAME_NAME;
		this->lidarSub = nh.subscribe(DEFAULT_SCAN_TOPIC, 1, &ObstacleDetector::lidarCallback,this);
//		this->obstaclePublisher = nh.advertise<geometry_msgs::PoseArray>("obstaclePoses", 1);

	//	this->run();
}

void ObstacleDetector::lidarCallback(const sensor_msgs::LaserScanConstPtr& input)
{
	this->lidarInput = *input;
	this->inpHeader = (*input).header;
	this->run();
}

static bool wayToSort(tf::Vector3 i, tf::Vector3 j)
{
	//bool v = i.quality<j.quality;
	return (i.getX()<j.getX());
}

void ObstacleDetector::removeDuplicates(std::vector<tf::Vector3>& worldLidarPoses)
{
	std::sort(worldLidarPoses.begin(), worldLidarPoses.end(), wayToSort);
	int cur = 0;
	int next = cur + 1;
	while(next < worldLidarPoses.size())
	{
		if((fabs(worldLidarPoses[next].getX() - worldLidarPoses[cur].getX()) < OBSTACLE_SPACE_THRESHOLD )&&
		   (fabs(worldLidarPoses[next].getY() - worldLidarPoses[cur].getY()) < OBSTACLE_SPACE_THRESHOLD)	)
		{
//			ROS_WARN_STREAM("Removed:"<<worldLidarPoses[cur].getX()<<" "<<worldLidarPoses[cur].getY()<<":: "<<worldLidarPoses[next].getX()<<" "<<worldLidarPoses[next].getY());
			worldLidarPoses.erase(worldLidarPoses.begin()+next);
		}
		else
		{
//			ROS_WARN_STREAM("NOT Removed:"<<worldLidarPoses[cur].getX()<<" "<<worldLidarPoses[cur].getY()<<":: "<<worldLidarPoses[next].getX()<<" "<<worldLidarPoses[next].getY());
			cur++;
			next++;
		}
	}

//	ROS_WARN_STREAM("Number of obstacles found:" << worldLidarPoses.size());

}

void ObstacleDetector::run()
{
	static tf::TransformListener listener;
		tf::StampedTransform worldToLidar;
		try
		{
			listener.lookupTransform(this->world_frame, this->lidar_frame, ros::Time(ros::Time(0)), worldToLidar);
		}
		catch(tf::TransformException &e)
		{
			ROS_WARN_STREAM(e.what());
		}

		tf::Transform lidarToWorld = worldToLidar.inverse();

		std::vector<tf::Vector3> lidarPoses;
		lidarPoses.reserve(this->lidarInput.ranges.size());


		for(int i=0; i<this->lidarInput.ranges.size(); ++i)
		{
			lidarPoses.push_back(tf::Vector3(lidarInput.ranges.at(i)*cos(lidarInput.angle_min + lidarInput.angle_increment*i),
											 lidarInput.ranges.at(i)*sin(lidarInput.angle_min + lidarInput.angle_increment*i),
											 0));
		}

		std::vector<tf::Vector3> worldLidarPoses;
//		worldLidarPoses.reserve(lidarPoses.size());
		for(auto e: lidarPoses)
		{
			tf::Vector3 temp = worldToLidar * e;
			if((temp.getZ() >= 0.3 && temp.getZ() <= 2.0) &&
			   (temp.getX() >= -10.0 && temp.getX() <= 10.0) &&
			   (temp.getY() >= -10.0 && temp.getY() <= 10.0))
			{
				worldLidarPoses.push_back(temp);
			}
		}

		removeDuplicates(worldLidarPoses);
//		ROS_INFO_STREAM("CALLING AGAIN");
//		removeDuplicates(worldLidarPoses);
//		ROS_INFO_STREAM("CALLING 3rd");
//		removeDuplicates(worldLidarPoses);

		geometry_msgs::Pose temp;
		obstaclePoses.poses.clear();
		obstaclePoses.header = inpHeader;


		for(auto e:worldLidarPoses)
		{
				temp.position.x = e.getX();
				temp.position.y = e.getY();
				temp.position.z = e.getZ();
				obstaclePoses.poses.push_back(temp);
//				ROS_INFO_STREAM("Obs Found: "<<e.getX()<<", "<<e.getY()<<", "<<e.getZ());
		}
		//TODO: Account for rotation time of lidar
//		ROS_INFO_STREAM("Size:"<<obstaclePoses.poses.size());
//		ROS_INFO_STREAM("Lidar pos:"<<lidarToWorld.getOrigin().getX()<<" "<<lidarToWorld.getOrigin().getY());
//		this->obstaclePublisher.publish(obstaclePoses);
}
