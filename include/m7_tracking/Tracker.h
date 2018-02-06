/*
 * Tracker.h
 *
 *  Created on: Feb 2, 2018
 *      Author: boz
 */

#ifndef M7_TRACKING_INCLUDE_M7_TRACKING_TRACKER_H_
#define M7_TRACKING_INCLUDE_M7_TRACKING_TRACKER_H_

#include "Params.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/video.hpp"
#include <vector>
#include <string>

class Tracker {
public:
	Tracker();
	virtual ~Tracker();


	void run(cv::Mat inputImg, int target_LowHue, int target_LowSat, int target_LowValue,int target_HighHue, int target_HighSat, int target_HighValue);

};



#endif /* M7_TRACKING_INCLUDE_M7_TRACKING_TRACKER_H_ */
