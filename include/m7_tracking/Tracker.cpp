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

void Tracker::displayTargets()
{
		cv::Mat display = this->inputImg;
		for(int i=0; i<imgRoombaPoses.size(); ++i)
		{
			cv::circle(display,Point(imgRoombaPoses[i].x,imgRoombaPoses[i].y),5,Scalar(0,255,0),5);
		}

//		imgRoombaPoses.clear();

	    cv::imshow("RoombaPoses Targets", display);
	    cv::waitKey(5);
}

/*
 * This is a development function for finding optimal HSV thresholds for blob detection.
 */
void Tracker::run(cv::Mat inputImg, int target_LowHue, int target_LowSat, int target_LowValue,int target_HighHue, int target_HighSat, int target_HighValue){

	cv::Mat imgHSV;
	cv::Mat imgThresholded;

	cv::cvtColor(inputImg, imgHSV, cv::COLOR_BGR2HSV);

	cv::inRange(imgHSV, Scalar(target_LowHue, target_LowSat, target_LowValue),Scalar(target_HighHue, target_HighSat, target_HighValue), imgThresholded);

	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8)));
	cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8,8)));

	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)) );
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6)) );

	cv::Mat imgCanny;
	std::vector<vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::Canny(imgThresholded, imgCanny, CANNY_THRESHOLD, CANNY_THRESHOLD*2);
	cv::findContours( imgCanny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Moments>  oMoments(contours.size());

		for(int i=0; i<contours.size(); ++i)
		{
			oMoments[i] = cv::moments(contours[i]);
		}

		for(int i=0; i<contours.size(); ++i)
		{

			if(oMoments[i].m00 > 500)
			{
				imgRoombaPoses.push_back(Point2f(oMoments[i].m10 / oMoments[i].m00 , oMoments[i].m01 / oMoments[i].m00));
			}
		}

		displayTargets();



}

