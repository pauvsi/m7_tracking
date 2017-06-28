#include "controller.hpp"


Controller::Controller()
{
	redTargetTracker[0] = *(new Tracker(CAMERA_TOPIC_1, CAMERA_FRAME_1, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
	redTargetTracker[1] = *(new Tracker(CAMERA_TOPIC_2, CAMERA_FRAME_2, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
	redTargetTracker[2] = *(new Tracker(CAMERA_TOPIC_3, CAMERA_FRAME_3, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
	redTargetTracker[3] = *(new Tracker(CAMERA_TOPIC_4, CAMERA_FRAME_4, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));
	redTargetTracker[4] = *(new Tracker(CAMERA_TOPIC_5, CAMERA_FRAME_5, REDILOWHUE, REDIHIGHHUE,
								REDILOWSATURATION, REDIHIGHSATURATION, REDILOWVALUE, REDIHIGHVALUE));

	greenTargetTracker[0] = *(new Tracker(CAMERA_TOPIC_1, CAMERA_FRAME_1, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));
	greenTargetTracker[1] = *(new Tracker(CAMERA_TOPIC_2, CAMERA_FRAME_2, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));
	greenTargetTracker[2] = *(new Tracker(CAMERA_TOPIC_3, CAMERA_FRAME_3, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));
	greenTargetTracker[3] = *(new Tracker(CAMERA_TOPIC_4, CAMERA_FRAME_4, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));
	greenTargetTracker[4] = *(new Tracker(CAMERA_TOPIC_5, CAMERA_FRAME_5, GREENILOWHUE, GREENIHIGHHUE,
										GREENILOWSATURATION, GREENIHIGHSATURATION, GREENILOWVALUE, GREENIHIGHVALUE));

	// INITIALIZE KALMAN FILTERS
	//0-4 for red 5-9 green
	for(int i=0; i<10; ++i)
		targets[i] = *(new GroundVehicle());


	this->init();
}

//TODO: check if initialization is right!!
void Controller::init()
{
	getReadings();
	removeCopies();

	for(int i=0; i<5; i++)
	{
//		double scalars[4]; scalars[0] = uniqueRedPoses[i].getX(); scalars[1] = uniqueRedPoses[i].getY(); scalars[2] = scalars[3] = 0;
//		targets[i].init(imageHeader.stamp.sec, Eigen::Matrix<double, 4, 1>::Matrix(scalars));

//		scalars[0] = uniqueGreenPoses[i].getX(); scalars[1] = uniqueGreenPoses[i].getY();
		targets[i].init(imageHeader.stamp.sec, Eigen::Matrix<double, 4, 1>(uniqueRedPoses[i].getX(), uniqueRedPoses[i].getY(), 0, 0));
		targets[i+5].init(imageHeader.stamp.sec, Eigen::Matrix<double, 4, 1>(uniqueGreenPoses[i].getX(), uniqueGreenPoses[i].getY(), 0, 0));
	}
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

	uniqueRedPoses.clear(); uniqueGreenPoses.clear();

	for(int i=0; i<5; ++i)
	{
		for(auto& e: redTargetTracker[i].getPoses())
		{
			uniqueRedPoses.push_back(e);
		}
		for(auto& e: greenTargetTracker[i].getPoses())
		{
			uniqueGreenPoses.push_back(e);
		}

	}

	this->imageHeader = redTargetTracker[4].getHeader();
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

	while(next < uniqueRedPoses.size())
	{
				//Dont need fabs for x since it's sorted
		if(! ((uniquePoses[next].getX() - uniquePoses[cur].getX() < SPACE_BETWEEN_ROOMBA) &&
			    fabs(uniquePoses[next].getY() - uniquePoses[cur].getY()) < SPACE_BETWEEN_ROOMBA))
		{
			cur++;
			next++;
		}
		else
		{
			if(next+1 < uniquePoses.size())
			{
				if(! ((uniquePoses[next+1].getX() - uniquePoses[next].getX() < SPACE_BETWEEN_ROOMBA) &&
						fabs(uniquePoses[next+1].getY() - uniquePoses[next].getY() < SPACE_BETWEEN_ROOMBA)) )
				{
					uniquePoses[cur].setX((uniquePoses[cur].getX() +
										   uniquePoses[next].getX() ) / 2);
					uniquePoses[cur].setY((uniquePoses[cur].getX() +
									       uniquePoses[next].getX() ) / 2);

					uniquePoses.erase(uniquePoses.begin()+next);
				}

				else
				{
					uniquePoses[cur].setX((uniquePoses[cur].getX() +
									   	   uniquePoses[next].getX() +
										   uniquePoses[next+1].getX()) / 3);
					uniquePoses[cur].setY((uniquePoses[cur].getY() +
										   uniquePoses[next].getY() +
										   uniquePoses[next+1].getY()) / 3);

					uniquePoses.erase(uniquePoses.begin()+next);
					uniquePoses.erase(uniquePoses.begin()+next+1);
				}
			}

			else
			{
				uniquePoses[cur].setX((uniquePoses[cur].getX() +
									   uniquePoses[next].getX() ) / 2);
				uniquePoses[cur].setY((uniquePoses[cur].getX() +
									   uniquePoses[next].getX() ) / 2);

				uniquePoses.erase(uniquePoses.begin()+next);
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
}

void Controller::updatePos()
{
	//Go through unique poses, find matching positions and update the Kalman Filters
	//What if I don't get a measurement for a roomba? ANS: Find out through empty list

	float min = FLT_MAX;
	float displacement;
	std::vector<geometry_msgs::PoseStamped, 5> currPose;
	int posIndex, targetIndex;

	//RED
	for(int i=0; i<5; ++i)
		currPose[i] = targets[i].getPoseStamped();

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

		targets[targetIndex].update(Eigen::Matrix<double, 2, 1>(uniqueRedPoses[posIndex].getX(), uniqueRedPoses[posIndex].getY()), imageHeader);
		uniqueRedPoses.erase(uniqueRedPoses.begin()+posIndex);
		currPose.erase(currPose.begin()+targetIndex);
	}

	//RED
	for(int i=0; i<5; ++i)
		currPose[i] = targets[i+5].getPoseStamped();

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

		targets[targetIndex+5].update(Eigen::Matrix<double, 2, 1>(uniqueGreenPoses[posIndex].getX(), uniqueGreenPoses[posIndex].getY()), imageHeader);
		uniqueGreenPoses.erase(uniqueGreenPoses.begin()+posIndex);
		currPose.erase(currPose.begin()+targetIndex);
	}

}
