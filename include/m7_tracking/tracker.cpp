/*
 *tracker.cpp
 *
 *  Created On: Nov 9, 2016
 *      Author: Logesh Roshan Ramadoss
 */

#include "tracker.hpp"
using std::vector;
using cv::Mat;
using cv::Point;
using cv::Vec4i;
using cv::Moments;
using cv::Point2f;
using cv::Scalar;

/*
 * Add topic for each roomba
 * make sure current image time is latest since the buffer is 2.
 * Make sure matrix initialization is right.
 * Unit tests
 *
 */





// For Testing Purposes
Tracker::Tracker()
{
	this->camera_frame = DEFAULT_CAMERA_FRAME_NAME;
	this->world_frame = DEFAULT_WORLD_FRAME_NAME;
	this->odom_frame = DEFAULT_ODOM_FRAME_NAME;
	this->readROSParameters();

	image_transport::ImageTransport it(nh);

	this->cameraSub = it.subscribeCamera(this->getCameraTopic(), 2, &Tracker::cameraCallback, this);

	ROS_DEBUG_STREAM(cameraSub.getInfoTopic());
	ROS_DEBUG_STREAM(cameraSub.getTopic());
	ROS_DEBUG_STREAM(cameraSub.getTransport());

	target_LowHue = 0;
	target_HighHue = 8;
	target_LowSat = 165;
	target_HighSat = 256;
	target_LowValue = 105;
	target_HighValue = 256;

	currentImgTime = DBL_MIN;
	lastImgTime = DBL_MIN;
	curImgOlder = false;

	ROS_DEBUG_STREAM(" Initialization Complete");
}

Tracker::Tracker(std::string cameraTopic, std::string cameraFrame, int target_LowHue, int target_HighHue, int target_LowSat, int target_HighSat,
		int target_LowValue, int target_HighValue)
{
	this->cameraTopic = cameraTopic;
	this->camera_frame = cameraFrame;
	this->world_frame = DEFAULT_WORLD_FRAME_NAME;
	this->odom_frame = DEFAULT_ODOM_FRAME_NAME;
	this->target_LowHue = target_LowHue;
	this->target_HighHue = target_HighHue;
	this->target_LowSat = target_LowSat;
	this->target_HighSat = target_HighSat;
	this->target_LowValue = target_LowValue;
	this->target_HighValue = target_HighValue;

	image_transport::ImageTransport it(nh);

	this->cameraSub = it.subscribeCamera(this->getCameraTopic(), 2, &Tracker::cameraCallback, this);

	ROS_DEBUG_STREAM(cameraSub.getInfoTopic());
	ROS_DEBUG_STREAM(cameraSub.getTopic());
	ROS_DEBUG_STREAM(cameraSub.getTransport());

	currentImgTime = DBL_MIN;
	lastImgTime = DBL_MIN;
	curImgOlder = false;

	ROS_DEBUG_STREAM("Done");
}

void Tracker::cameraCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam)
{
	this->imageHeader =	img->header;
	lastImgTime = currentImgTime;
	currentImgTime = img->header.stamp.toNSec();
	if(currentImgTime < lastImgTime)
	{
		currentImgTime = lastImgTime;
		curImgOlder = true;
	}
	else
	{
		curImgOlder = false;
	}
	ROS_DEBUG_STREAM("Listening To Camera: In callback");

	//set the K and D matrices
	this->setK(get3x3FromVector(cam->K));
	this->setD(cv::Mat(cam->D, false));
	this->inputImg = cv_bridge::toCvShare(img, "bgr8")->image.clone();
	this->run();

}

cv::Mat Tracker::get3x3FromVector(boost::array<double, 9> vec)
{
	cv::Mat mat = cv::Mat(3,3, CV_32F);
	for(int i=0; i<3; ++i)
	{
		mat.at<float>(i, 0) = vec.at(3 * i + 0);
		mat.at<float>(i, 1) = vec.at(3 * i + 1);
		mat.at<float>(i, 2) = vec.at(3 * i + 2);
	}

	//ROS_DEBUG_STREAM_ONCE("K = " << mat);
	return mat;
}

void Tracker::readROSParameters()
{
	//CAMERA TOPIC
	ROS_WARN_COND(!ros::param::has("~cameraTopic"), "Parameter for 'cameraTopic' has not been set");
	ros::param::param<std::string>("~cameraTopic", cameraTopic, DEFAULT_CAMERA_TOPIC);
	ROS_DEBUG_STREAM("camera topic is: " << cameraTopic);

	ros::param::param<std::string>("~camera_frame_name", camera_frame, DEFAULT_CAMERA_FRAME_NAME);
	ros::param::param<std::string>("~odom_frame_name", odom_frame, DEFAULT_ODOM_FRAME_NAME);
	ros::param::param<std::string>("~world_frame_name", world_frame, DEFAULT_WORLD_FRAME_NAME);
}

void Tracker::displayTargets()
{
		cv::Mat display = this->inputImg;
		for(int i=0; i<imgRoombaPoses.size(); ++i)
		{
			cv::circle(display,Point(imgRoombaPoses[i].x,imgRoombaPoses[i].y),5,Scalar(0,255,0),2);
		}

		imgRoombaPoses.clear();

	    cv::imshow("RoombaPoses Targets", display);
	    cv::waitKey(5);
}

//Find Region of Target in ColorSpace
void Tracker::createTrackBars()
{
	const std::string trackbarWindowName = "Trackbars";

    cv::namedWindow(trackbarWindowName, CV_WINDOW_AUTOSIZE);

    cv::waitKey(30);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	std::sprintf( TrackbarName, "H_MIN", target_LowHue);
	std::sprintf( TrackbarName, "H_MAX", target_HighHue);
	std::sprintf( TrackbarName, "S_MIN", target_LowSat);
	std::sprintf( TrackbarName, "S_MAX", target_HighSat);
	std::sprintf( TrackbarName, "V_MIN", target_LowValue);
	std::sprintf( TrackbarName, "V_MAX", target_HighValue);


    cv::createTrackbar( "H_MIN", trackbarWindowName, &target_LowHue, target_HighHue);
    cv::createTrackbar( "H_MAX", trackbarWindowName, &target_HighHue, target_HighHue);
    cv::createTrackbar( "S_MIN", trackbarWindowName, &target_LowSat, target_HighSat);
    cv::createTrackbar( "S_MAX", trackbarWindowName, &target_HighSat, target_HighSat);
    cv::createTrackbar( "V_MIN", trackbarWindowName, &target_LowValue, target_HighValue);
    cv::createTrackbar( "V_MAX", trackbarWindowName, &target_HighValue, target_HighValue);

}

void Tracker::run()
{

	cv::Mat imgHSV;
	cv::Mat imgThresholded;

	cv::cvtColor(inputImg, imgHSV, cv::COLOR_BGR2HSV);
	//TODO: GET HSV Range for the Red Roomba. (IMPORTANT!)
	cv::inRange(imgHSV, Scalar(target_LowHue, target_LowSat, target_LowValue),
						Scalar(target_HighHue, target_HighSat, target_HighValue), imgThresholded);

/*	cv::imshow("Thresholded image", imgThresholded);
	cv::waitKey(30);
	return;
*/

	//Remove small objects from the foreground (Morphological Opening)
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8)));
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8)));

	//Fill out small holes in the background (Morphological Closing)
	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)) );
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)) );

	cv::Mat imgCanny;
	std::vector<vector<Point> > contours;
	std::vector<Vec4i> hierarchy;

	cv::Canny(imgThresholded, imgCanny, CANNY_THRESHOLD, CANNY_THRESHOLD*2);
	cv::findContours( imgCanny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);



	//Calculate the Moments
	std::vector<cv::Moments>  oMoments(contours.size());

	for(int i=0; i<contours.size(); ++i)
	{
		oMoments[i] = cv::moments(contours[i]);
	}

	for(int i=0; i<contours.size(); ++i)
	{
		//*******************Ensure it's a roomba. Might be unnecessary
		if(oMoments[i].m00 > 500)  //******************************************** Check Size **********************
		{
			imgRoombaPoses.push_back(Point2f(oMoments[i].m10 / oMoments[i].m00 , oMoments[i].m01 / oMoments[i].m00));
			ROS_WARN_STREAM("Position of"<< i+1<< "Roomba is: x:" << imgRoombaPoses[i].x << " y:"<< imgRoombaPoses[i].y << std::endl);
		}
	}


		displayTargets();
		getWorldPosition();

		return;


}



void Tracker::removeOutofBounds()
{
	for(int  i=0; i < worldRoombaPosition.size(); ++i)
	{
		if((worldRoombaPosition[i].getX() >= -10.0 && worldRoombaPosition[i].getX() <= 10.00) &&
		   (worldRoombaPosition[i].getY() >= -10.0 && worldRoombaPosition[i].getY() <=10.00))
		{

		}
		else
		{
			worldRoombaPosition.erase(worldRoombaPosition.begin()+i);
		}
	}
}

void Tracker::getWorldPosition()
{
	static tf::TransformListener listener;
	tf::StampedTransform worldToCam;

	//NOTE: If you multiply WorldToCam with a pos in world frame, then you'll get a pos in cam coord frame
	try
	{
		listener.lookupTransform(this->camera_frame, this->world_frame, ros::Time(ros::Time(0)), worldToCam);
	}
	catch(tf::TransformException &e)
	{
		ROS_WARN_STREAM(e.what());
	}

	// [u v]
//	vector<cv::Point2f> undistortedPoses;
/*
	if(!FISHEYE_DISTORTION)
		cv::undistortPoints(imgRoombaPoses, undistortedPoses, this->K, this->D);
	else
		cv::fisheye::undistortPoints(imgRoombaPoses, undistortedPoses, this->K, this->D);
*/

	// [x y z] projected from camera onto plane, 1m away from lens
	vector<tf::Vector3> projectedPoses;

	for(auto e : imgRoombaPoses)
	{
		projectedPoses.push_back(tf::Vector3(K.inv()*tf::Vector3(e.x, e.y, 1)));
	}
/*
	for(int i=0; i<undistortedPoses.size(); ++i)
	{
		projectedPoses.push_back(tf::Vector3(undistortedPoses[i].x, undistortedPoses[i].y, 1));
	}
*/

	//[x y z] of projected points in world coordinate frame
	vector<tf::Vector3> worldProjectedPoses;
	tf::Vector3 cameraPos = worldToCam.getOrigin(); //Origin of cam in world frame
	tf::Transform camToWorld = worldToCam.inverse();


	for(auto e : projectedPoses)
	{
		worldProjectedPoses.push_back(camToWorld * (tf::Vector3(e))); // will give point in woorld coord
	}


	tf::Vector3 lineVector; //vector along line [a b c]
	double lineParameter; // parameter t for line vector

	for(auto e: worldProjectedPoses)
	{
		//r = r. + tv
		//V = [a b c]
		lineVector = e - cameraPos;
		// t = (z-z.)/c
		lineParameter = (ROOMBA_HEIGHT - cameraPos.getZ())/lineVector.getZ();

		//Due to vision, one roomba could have two positions. Remove this by looking at
		//roomba size and averaging the nearby points into one roomba position

		worldRoombaPosition.push_back(tf::Vector3(cameraPos.getX() + lineParameter*lineVector.getX(),
													cameraPos.getY() + lineParameter*lineVector.getY(),
													ROOMBA_HEIGHT));
	}

	removeOutofBounds();

	for(int i = 0; i<worldRoombaPosition.size(); ++i)
	{
		ROS_WARN_STREAM("\n Roomba "<<i+1<<" World Position: "<< worldRoombaPosition[i]);
	}


	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	q.setRPY(0,0,0);
	transform.setRotation(q);

	for(int i=0; i<worldRoombaPosition.size(); ++i)
	{
		transform.setOrigin(worldRoombaPosition[i]);


		br.sendTransform(tf::StampedTransform(transform, imageHeader.stamp, this->world_frame,
						std::string(this->camera_frame + " Roomba " + boost::lexical_cast<std::string>(i))));
//		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->world_frame,
//								buffer));
	}

}
