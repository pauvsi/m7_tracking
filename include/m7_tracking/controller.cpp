#include "controller.h"
#include <math.h>
/*
 * TODO: in init. Chances are i wont get all 10 roombas. What do I do?
 */

Controller::~Controller()
{

}
Controller::Controller()
{
	ROS_INFO_STREAM("CONSTRUCTOR");
	redTargetTracker.push_back((new Tracker(CAMERA_TOPIC_1, CAMERA_FRAME_1, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE)));
//	redTargetTracker.push_back(*(new Tracker(CAMERA_TOPIC_2, CAMERA_FRAME_2, REDILOWHUE, REDIHIGHHUE,
//								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE)));
//	redTargetTracker.push_back ( *(new Tracker(CAMERA_TOPIC_3, CAMERA_FRAME_3, REDILOWHUE, REDIHIGHHUE,
//								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE)));
//	redTargetTracker.push_back( *(new Tracker(CAMERA_TOPIC_4, CAMERA_FRAME_4, REDILOWHUE, REDIHIGHHUE,
//								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE)));
//	redTargetTracker.push_back( *(new Tracker(CAMERA_TOPIC_5, CAMERA_FRAME_5, REDILOWHUE, REDIHIGHHUE,
//								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE)));

	greenTargetTracker.push_back( (new Tracker(CAMERA_TOPIC_1, CAMERA_FRAME_1, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE)));
//	greenTargetTracker.push_back( *(new Tracker(CAMERA_TOPIC_2, CAMERA_FRAME_2, GREENILOWHUE, GREENIHIGHHUE,
//										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE)));
//	greenTargetTracker.push_back(*(new Tracker(CAMERA_TOPIC_3, CAMERA_FRAME_3, GREENILOWHUE, GREENIHIGHHUE,
//										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE)));
//	greenTargetTracker.push_back(*(new Tracker(CAMERA_TOPIC_4, CAMERA_FRAME_4, GREENILOWHUE, GREENIHIGHHUE,
//										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE)));
//	greenTargetTracker.push_back(*(new Tracker(CAMERA_TOPIC_5, CAMERA_FRAME_5, GREENILOWHUE, GREENIHIGHHUE,
//										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE)));


	// INITIALIZE KALMAN FILTERS
	//0-4 for red 5-9 green
	for(int i=0; i<10; ++i)
		targets.push_back( new GroundVehicle());

	for(int i=0; i<5; ++i)
		obstacles.push_back( new GroundVehicle());

	red1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba1", 1);
	red2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba2", 1);
	red3 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba3", 1);
	red4 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba4", 1);
	red5 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba5", 1);
	green1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba6", 1);
	green2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba7", 1);
	green3 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba8", 1);
	green4 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba9", 1);
	green5 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("roomba/roomba10", 1);

	obs1 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("obstacle/obstacle1", 1);
	obs2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("obstacle/obstacle2", 1);
	obs3 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("obstacle/obstacle3", 1);
	obs4 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("obstacle/obstacle4", 1);

	initialized = false;
	redTargetCtr = greenTargetCtr = obsCtr = 0;
	fullInit = false;
//	this->init();


}

//TODO: check if initialization is right!!
void Controller::init()
{
	ROS_INFO_STREAM("INIT");
	//TODO: IF IM IN THIS FUNCTION, I CANT GO TO A CALLBACK?

	getReadings();

	removeCopies();
	if(uniqueRedPoses.size()==0 && uniqueGreenPoses.size() == 0)
		return;


	for(auto& e:uniqueRedPoses)
	{
		(*targets[redTargetCtr]).init(imageHeader, Eigen::Matrix<double, 4, 1> (e.getX(), e.getY(), 0, 0));
		++redTargetCtr;
	}

	for(auto& e:uniqueGreenPoses)
	{
		(*targets[greenTargetCtr+5]).init(imageHeader, Eigen::Matrix<double, 4, 1> (e.getX(), e.getY(), 0, 0));
		++greenTargetCtr;
	}

//	for(int i=0; i<5; i++)
//	{
////		double scalars[4]; scalars[0] = uniqueRedPoses[i].getX(); scalars[1] = uniqueRedPoses[i].getY(); scalars[2] = scalars[3] = 0;
////		targets[i].init(imageHeader.stamp.sec, Eigen::Matrix<double, 4, 1>::Matrix(scalars));
//
////		scalars[0] = uniqueGreenPoses[i].getX(); scalars[1] = uniqueGreenPoses[i].getY();
//		(*targets[i]).init(imageHeader.stamp.toSec(), Eigen::Matrix<double, 4, 1>  (uniqueRedPoses[i].getX(), uniqueRedPoses[i].getY(), 0, 0));
//		(*targets[i+5]).init(imageHeader.stamp.toSec(), Eigen::Matrix<double, 4, 1> (uniqueGreenPoses[i].getX(), uniqueGreenPoses[i].getY(), 0, 0));
//		ROS_INFO_STREAM("TARGETS INITIALIZED");
//
//	}

	ROS_INFO_STREAM("TARGETS INITIALIZED");
//	for(int i=0; i<4; ++i)
//	{
//		(*obstacles[i]).init(imageHeader.stamp.toSec(), Eigen::Matrix<double, 4, 1> ( uniqueObstaclePoses[i].getX(), uniqueObstaclePoses[i].getY(), 0,0));
//	}

	ROS_INFO_STREAM("init done");
	ROS_WARN_STREAM("ctrs:"<<redTargetCtr<<" "<<greenTargetCtr);
	initialized = true;
}

void Controller::updateAndInit()
{
	getReadings();
	removeCopies();
	if(uniqueRedPoses.size()==0 && uniqueGreenPoses.size() == 0)
		return;

	//Go through unique poses, find matching positions and update the Kalman Filters
	//What if I don't get a measurement for a roomba? ANS: Find out through empty list

	float min = FLT_MAX;
	float threshold = 1.0;
	float displacement;
	bool flag = false;
	std::vector<geometry_msgs::PoseStamped> currPose;
	int posIndex, targetIndex;

	ROS_INFO_STREAM("Time:"<<imageHeader.stamp.toSec());

//TODO: THIS PREDICT WITH THE PREDICTION MIGHT BE A PROBLEM
	//RED
	for(int i=0; i<redTargetCtr; ++i)
	{
		currPose.push_back((*targets[i]).getPoseStamped(imageHeader));
		ROS_INFO_STREAM("Pushed:"<<currPose[i].pose.position.x<<currPose[i].pose.position.y);
		if(std::isnan(currPose[i].pose.position.x ))
			std::exit(0);
	}

	int temp = uniqueRedPoses.size();
	for(int i=0; i<temp; ++i)
	{
		if(uniqueRedPoses.empty())
			break;

		flag = false;
		min = FLT_MAX;
		for(int k=0; k<currPose.size();++k)
		{
			int j = 0;
			while(j < uniqueRedPoses.size())
			{
				displacement = sqrt((currPose[k].pose.position.x - uniqueRedPoses[j].getX())*
								(currPose[k].pose.position.x - uniqueRedPoses[j].getX()) +
									(currPose[k].pose.position.y - uniqueRedPoses[j].getY())*
									(currPose[k].pose.position.y - uniqueRedPoses[j].getY()));
				if((displacement < threshold) && flag == false)
				{
					flag = true;
					min = displacement;
					posIndex = j;
					targetIndex = k;
				}

				if((displacement < min) && flag == true)
				{
					min = displacement;
					posIndex = j;
					targetIndex = k;
				}
				++j;
			}
		}

		if(flag == false)
			continue;

		ROS_INFO_STREAM("UPDATED: KF:"<<currPose[targetIndex].pose.position.x<<" "<<currPose[targetIndex].pose.position.y<<" Red:" <<uniqueRedPoses[posIndex].getX()<<" "<<uniqueRedPoses[posIndex].getY());
		(*targets[targetIndex]).update(Eigen::Matrix<double, 2, 1>(uniqueRedPoses[posIndex].getX(), uniqueRedPoses[posIndex].getY()), imageHeader);
		uniqueRedPoses.erase(uniqueRedPoses.begin()+posIndex);
		currPose.erase(currPose.begin()+targetIndex);
	}


	for(auto&e : uniqueRedPoses)
	{
		(*targets[redTargetCtr]).init(imageHeader, Eigen::Matrix<double, 4, 1> (e.getX(), e.getY(), 0, 0));
		++redTargetCtr;
		ROS_INFO_STREAM("Made nwe KF: "<<" Red:" <<e.getX()<<" "<<e.getY());
		for(auto&f : currPose)
			ROS_WARN_STREAM(" \t"<<f.pose.position.x << f.pose.position.y);
		if(redTargetCtr == 5)
			break;
	}

	currPose.clear();


	//GREEN
	for(int i=0; i<greenTargetCtr; ++i)
		currPose.push_back( (*targets[i+5]).getPoseStamped(imageHeader));

	temp = uniqueGreenPoses.size();
	for(int i=0; i<temp; ++i)
	{
		if(uniqueGreenPoses.empty())
			break;

		flag = false;
		min = FLT_MAX;
		for(int k=0; k<currPose.size();++k)
		{
			int j = 0;
			while(j < uniqueGreenPoses.size())
			{
				displacement = sqrt((currPose[k].pose.position.x - uniqueGreenPoses[j].getX())*
								(currPose[k].pose.position.x - uniqueGreenPoses[j].getX()) +
									(currPose[k].pose.position.y - uniqueGreenPoses[j].getY())*
									(currPose[k].pose.position.y - uniqueGreenPoses[j].getY()));
				if((displacement < threshold) && flag == false)
				{
					flag = true;
					min = displacement;
					posIndex = j;
					targetIndex = k;
				}

				if((displacement < min) && flag == true)
				{
					min = displacement;
					posIndex = j;
					targetIndex = k;
				}
				++j;
			}
		}

		if(flag == false)
			continue;

		ROS_INFO_STREAM("UPDATED");
		(*targets[targetIndex+5]).update(Eigen::Matrix<double, 2, 1>(uniqueGreenPoses[posIndex].getX(), uniqueGreenPoses[posIndex].getY()), imageHeader);
		uniqueGreenPoses.erase(uniqueGreenPoses.begin()+posIndex);
		currPose.erase(currPose.begin()+targetIndex);
	}

	for(auto& e:uniqueGreenPoses)
	{
		(*targets[greenTargetCtr+5]).init(imageHeader, Eigen::Matrix<double, 4, 1> (e.getX(), e.getY(), 0, 0));
		++greenTargetCtr;
		ROS_INFO_STREAM("Made nwe KF");

		if(greenTargetCtr == 5)
			break;
	}


	if(greenTargetCtr + redTargetCtr >= 10)
	{
		fullInit = true;
		ROS_INFO_STREAM("Full init done!");
	}
	//MAKE SURE IT DOESNT GO OVER 10?
}


void Controller::getReadings()
{
//	std::vector<tf::Vector3> temp;
//	for(int i=0; i<5; ++i)
//	{
//		temp = redTargetTracker[i].getPoses();
//		uniqueRedPoses.insert(uniqueRedPoses.end(), temp.begin(), temp.end()); //(redTargetTracker[i].getPoses());
//
//		temp = greenTargetTracker[i].getPoses();
//		uniqueGreenPoses.insert(uniqueGreenPoses.end(), temp.begin(), temp.end());
//
//	}

	uniqueRedPoses.clear(); uniqueGreenPoses.clear(); uniqueObstaclePoses.clear();
/*
	for(int i=0; i<5; ++i)
	{
		redTargetTracker[i].run();
		greenTargetTracker[i].run();
	}
*/
	for(int i=0; i<redTargetTracker.size(); ++i)
	{
		if((*redTargetTracker[i]).curImgOlder)
			continue;

		for(auto& e: (*redTargetTracker[i]).getPoses())
		{
			uniqueRedPoses.push_back(e);
		}
		for(auto& e: (*greenTargetTracker[i]).getPoses())
		{
			uniqueGreenPoses.push_back(e);
		}

	}

	for(auto& e: obsDet.getPoseVector())
	{
		uniqueObstaclePoses.push_back(e);
	}

	this->imageHeader = (*redTargetTracker[0]).getHeader();

//	ROS_INFO_STREAM("Get Readings Done! Size:"<<uniqueRedPoses.size());
}

//returns true if the first argument goes before the second argument in the strict weak ordering
static bool wayToSort(tf::Vector3 i, tf::Vector3 j)
{
	//bool v = i.quality<j.quality;
	return (i.getX()<j.getX());
}


void Controller::mergeCopies(std::vector<tf::Vector3> &uniquePoses)
{
	int cur = 0;
	int next = cur+1;

	while(next < uniquePoses.size())
	{
				//Dont need fabs for x since it's sorted
		if(! ( (uniquePoses[next].getX() - uniquePoses[cur].getX() < SPACE_BETWEEN_ROOMBA) &&
			    (fabs(uniquePoses[next].getY() - uniquePoses[cur].getY()) < SPACE_BETWEEN_ROOMBA)))
		{
			cur++;
			next++;
		}
		else
		{
			if(next+1 < uniquePoses.size())
			{
				if(! ((uniquePoses[next+1].getX() - uniquePoses[next].getX() < SPACE_BETWEEN_ROOMBA) &&
						(fabs(uniquePoses[next+1].getY() - uniquePoses[next].getY() < SPACE_BETWEEN_ROOMBA)) ))
				{
					uniquePoses[cur].setX((uniquePoses[cur].getX() +
										   uniquePoses[next].getX() ) / 2);
					uniquePoses[cur].setY((uniquePoses[cur].getX() +
									       uniquePoses[next].getX() ) / 2);

					uniquePoses.erase(uniquePoses.begin()+(next));
				}

				else
				{
					uniquePoses[cur].setX((uniquePoses[cur].getX() +
									   	   uniquePoses[next].getX() +
										   uniquePoses[next+1].getX()) / 3);
					uniquePoses[cur].setY((uniquePoses[cur].getY() +
										   uniquePoses[next].getY() +
										   uniquePoses[next+1].getY()) / 3);

					uniquePoses.erase(uniquePoses.begin()+(next));
					uniquePoses.erase(uniquePoses.begin()+(next+1));
				}
			}

			else
			{
				uniquePoses[cur].setX((uniquePoses[cur].getX() +
									   uniquePoses[next].getX() ) / 2);
				uniquePoses[cur].setY((uniquePoses[cur].getX() +
									   uniquePoses[next].getX() ) / 2);

				uniquePoses.erase(uniquePoses.begin()+(next));
			}
		}
	}

	return;
}


void Controller::removeCopies()
{
	std::sort(uniqueRedPoses.begin(), uniqueRedPoses.end(), wayToSort);
	std::sort(uniqueGreenPoses.begin(), uniqueGreenPoses.end(), wayToSort);


	mergeCopies(uniqueRedPoses);
	mergeCopies(uniqueGreenPoses);

//	ROS_INFO_STREAM("removeCopies done");
}

void Controller::updateTargetPos()
{
	//Go through unique poses, find matching positions and update the Kalman Filters
	//What if I don't get a measurement for a roomba? ANS: Find out through empty list

	float min = FLT_MAX;
	float displacement;
	std::vector<geometry_msgs::PoseStamped> currPose(5);
	int posIndex, targetIndex;

//TODO: THIS PREDICT WITH THE PREDICTION MIGHT BE A PROBLEM
	//RED
	for(int i=0; i<5; ++i)
	{
		currPose[i] = (*targets[i]).getPoseStamped(imageHeader);
	}

	for(int i=0; i<5; ++i)
	{
		if(uniqueRedPoses.empty())
			break;

		min = FLT_MAX;
		for(int k=0; k<currPose.size();++k)
		{
			int j = 0;
			while(j < uniqueRedPoses.size())
			{
				displacement = (currPose[k].pose.position.x - uniqueRedPoses[j].getX())*
								(currPose[k].pose.position.x - uniqueRedPoses[j].getX()) +
									(currPose[k].pose.position.y - uniqueRedPoses[j].getY())*
									(currPose[k].pose.position.y - uniqueRedPoses[j].getY());
				if(displacement < min)
				{
					min = displacement;
					posIndex = j;
					targetIndex = k;
				}
				++j;
			}
		}

		(*targets[targetIndex]).update(Eigen::Matrix<double, 2, 1>(uniqueRedPoses[posIndex].getX(), uniqueRedPoses[posIndex].getY()), imageHeader);
		uniqueRedPoses.erase(uniqueRedPoses.begin()+posIndex);
		currPose.erase(currPose.begin()+targetIndex);
	}

	//GREEN
	for(int i=0; i<5; ++i)
		currPose[i] = (*targets[i+5]).getPoseStamped(imageHeader);

	for(int i=0; i<5; ++i)
	{
		if(uniqueGreenPoses.empty())
			break;

		min = FLT_MAX;
		for(int k=0; k<currPose.size();++k)
		{
			int j = 0;
			while(j < uniqueGreenPoses.size())
			{
				displacement = (currPose[k].pose.position.x - uniqueGreenPoses[j].getX())*
								(currPose[k].pose.position.x - uniqueGreenPoses[j].getX()) +
									(currPose[k].pose.position.y - uniqueGreenPoses[j].getY())*
									(currPose[k].pose.position.y - uniqueGreenPoses[j].getY());
				if(displacement < min)
				{
					min = displacement;
					posIndex = j;
					targetIndex = k;
				}
				++j;
			}
		}

		(*targets[targetIndex+5]).update(Eigen::Matrix<double, 2, 1>  (uniqueGreenPoses[posIndex].getX(), uniqueGreenPoses[posIndex].getY()), imageHeader);
		uniqueGreenPoses.erase(uniqueGreenPoses.begin()+posIndex);
		currPose.erase(currPose.begin()+targetIndex);
	}


}

void Controller::updateObsPos()
{
	float min = FLT_MAX;
		float displacement;
		std::vector<geometry_msgs::PoseStamped> currPose(4);
		int posIndex, targetIndex;

		//Obstacles
		for(int i=0; i<4; ++i)
			currPose[i] = (*targets[i]).getPoseStamped(imageHeader);

		for(int i=0; i<4; ++i)
			{
				if(uniqueObstaclePoses.empty())
					break;

				min = FLT_MAX;
				for(int k=0; k<currPose.size();++k)
				{
					int j = 0;
					while(j < uniqueObstaclePoses.size())
					{
						displacement = (currPose[k].pose.position.x - uniqueObstaclePoses[j].getX())*
										(currPose[k].pose.position.x - uniqueObstaclePoses[j].getX()) +
											(currPose[k].pose.position.y - uniqueObstaclePoses[j].getY())*
											(currPose[k].pose.position.y - uniqueObstaclePoses[j].getY());
						if(displacement < min)
						{
							min = displacement;
							posIndex = j;
							targetIndex = k;
						}
						++j;
					}
				}

				(*obstacles[targetIndex]).update(Eigen::Matrix<double, 2, 1> ( uniqueObstaclePoses[posIndex].getX(), uniqueObstaclePoses[posIndex].getY()), imageHeader);
				uniqueObstaclePoses.erase(uniqueObstaclePoses.begin()+posIndex);
				currPose.erase(currPose.begin()+targetIndex);
			}
}

void Controller::publishAll()
{
	red1.publish((*targets[0]).getPoseWithCovariance(imageHeader));
	red2.publish((*targets[1]).getPoseWithCovariance(imageHeader));
	red3.publish((*targets[2]).getPoseWithCovariance(imageHeader));
	red4.publish((*targets[3]).getPoseWithCovariance(imageHeader));
	red5.publish((*targets[4]).getPoseWithCovariance(imageHeader));
	green1.publish((*targets[5]).getPoseWithCovariance(imageHeader));
	green2.publish((*targets[6]).getPoseWithCovariance(imageHeader));
	green3.publish((*targets[7]).getPoseWithCovariance(imageHeader));
	green4.publish((*targets[8]).getPoseWithCovariance(imageHeader));
	green5.publish((*targets[9]).getPoseWithCovariance(imageHeader));

	obs1.publish((*obstacles[0]).getPoseWithCovariance(imageHeader));
	obs2.publish((*obstacles[1]).getPoseWithCovariance(imageHeader));
	obs3.publish((*obstacles[2]).getPoseWithCovariance(imageHeader));
	obs4.publish((*obstacles[3]).getPoseWithCovariance(imageHeader));
}


void Controller::run()
{
//	ROS_INFO_STREAM("IN RUN");
//	uniqueRedPoses.clear(); uniqueGreenPoses.clear(); uniqueObstaclePoses.clear();
	getReadings();
	removeCopies();
	updateTargetPos();
	updateObsPos();
	publishAll();
}
