/*
 * Tracker.cpp
 *
 *  Created on: Feb 2, 2018
 *      Author: boz
 */

#include "Tracker.h"


Tracker::Tracker() {
	// TODO Auto-generated constructor stub

}

Tracker::~Tracker() {
	// TODO Auto-generated destructor stub
}

void Tracker::displayTargets(std::vector<cv::Point> imgRoombaPoses, cv::Mat inputImg)
{
		cv::Mat display = inputImg;
		for(int i=0; i<imgRoombaPoses.size(); ++i)
		{
			ROS_INFO_STREAM("CIRCLE DRAWN");
			cv::circle(display,cv::Point(imgRoombaPoses[i].x,imgRoombaPoses[i].y),15,cv::Scalar(255,0,0),15);
		}

//		imgRoombaPoses.clear();

	    cv::imshow("RoombaPoses Targets", display);
	    cv::waitKey(5);
}

/*
 * This is a development function for finding optimal HSV thresholds for blob detection.
 */
void Tracker::run(const sensor_msgs::ImageConstPtr& inputImg){

	cv::Mat imgHSV;
	cv::Mat upperRed;
	cv::Mat lowerRed;
	cv::Mat imgThresholded;
	std::vector<cv::Point> imgRoombaPoses;

	//converts ross sensor_msg::img to cv:: Mat
	cv::Mat temp = cv_bridge::toCvShare(inputImg, inputImg->encoding)->image.clone();

	cv::cvtColor(temp, imgHSV, cv::COLOR_BGR2HSV);

	cv::inRange(imgHSV, cv::Scalar(RED_HUE_HSV_LOW_ONE, RED_SATURATION_HSV_LOW, RED_VALUE_HSV_LOW),cv::Scalar(RED_HUE_HSV_HIGH_ONE, RED_SATURATION_HSV_HIGH,  RED_VALUE_HSV_HIGH), upperRed);
	cv::inRange(imgHSV, cv::Scalar(RED_HUE_HSV_LOW_TWO, RED_SATURATION_HSV_LOW, RED_VALUE_HSV_LOW),cv::Scalar(RED_HUE_HSV_HIGH_TWO, RED_SATURATION_HSV_HIGH,  RED_VALUE_HSV_HIGH), lowerRed);
	cv::addWeighted(lowerRed, 1.0, upperRed, 1.0, 0.0, imgThresholded);

	//cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8)));
	//cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8)));

	cv::imshow("filter 1", imgThresholded);
	//cv::imshow("filter 2", lowerRed);
	cv::waitKey(5);

	//cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)) );
	//cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)) );

	//cv::imshow("filter 2",imgThresholded);
	//cv::waitKey(5);

	cv::Mat imgCanny;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::Canny(imgThresholded, imgCanny, CANNY_THRESHOLD, CANNY_THRESHOLD*2);
	cv::findContours(imgCanny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Moments>  oMoments(contours.size());

		for(int i=0; i<contours.size(); ++i)
		{
			oMoments[i] = cv::moments(contours[i]);
		}

		for(int i=0; i<contours.size(); ++i)
		{

			if(oMoments[i].m00 > 500)
			{
				ROS_INFO_STREAM("POSE ADDED");
				imgRoombaPoses.push_back(cv::Point2f(oMoments[i].m10 / oMoments[i].m00 , oMoments[i].m01 / oMoments[i].m00));
			}
		}



		//displayTargets(imgRoombaPoses,temp);



}

