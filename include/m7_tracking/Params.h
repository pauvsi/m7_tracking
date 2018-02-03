/*
 * Params.h
 *
 *  Created on: Feb 2, 2018
 *      Author: boz
 */

#ifndef M7_TRACKING_INCLUDE_M7_TRACKING_PARAMS_H_
#define M7_TRACKING_INCLUDE_M7_TRACKING_PARAMS_H_

#include <string.h>
#include <ros/ros.h>

#define D_CAMERA_IMAGE_TOPIC_1 "front_camera/image"
#define D_CAMERA_INFO_TOPIC_1 "front_camera_info"

#define D_CAMERA_IMAGE_TOPIC_2 "left_camera/image"
#define D_CAMERA_INFO_TOPIC_2 "left_camera_info"

#define D_CAMERA_IMAGE_TOPIC_3 "back_camera/image"
#define D_CAMERA_INFO_TOPIC_3 "back_camera_info"

#define D_CAMERA_IMAGE_TOPIC_4 "right_camera/image"
#define D_CAMERA_INFO_TOPIC_4 "right_camera_info"



extern std::string CAMERA_IMAGE_TOPIC_1;
extern std::string CAMERA_INFO_TOPIC_1;

extern std::string CAMERA_IMAGE_TOPIC_2;
extern std::string CAMERA_INFO_TOPIC_2;

extern std::string CAMERA_IMAGE_TOPIC_3;
extern std::string CAMERA_INFO_TOPIC_3;

extern std::string CAMERA_IMAGE_TOPIC_4;
extern std::string CAMERA_INFO_TOPIC_4;

#endif /* M7_TRACKING_INCLUDE_M7_TRACKING_PARAMS_H_ */
