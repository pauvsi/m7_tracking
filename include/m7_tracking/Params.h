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

#define D_CAMERA_IMAGE_TOPIC_2 "left_camera/image"

#define D_CAMERA_IMAGE_TOPIC_3 "back_camera/image"

#define D_CAMERA_IMAGE_TOPIC_4 "right_camera/image"

//hsv bounds
#define D_RED_HUE_HSV_HIGH 256
#define D_HUE_RED_HSV_LOW 0
#define D_SATURATION_RED_HSV_HIGH 256
#define D_SATURATION_RED_HSV_LOW 0
#define D_VALUE_RED_HSV_HIGH 256
#define D_VALUE_RED_HSV_LOW 0

#define D_GREEN_HUE_HSV_HIGH 256
#define D_HUE_GREEN_HSV_LOW 0
#define D_SATURATION_GREEN_HSV_HIGH 256
#define D_SATURATION_GREEN_HSV_LOW 0
#define D_VALUE_GREEN_HSV_HIGH 256
#define D_VALUE_GREEN_HSV_LOW 0

extern std::string CAMERA_IMAGE_TOPIC_1;

extern std::string CAMERA_IMAGE_TOPIC_2;

extern std::string CAMERA_IMAGE_TOPIC_3;

extern std::string CAMERA_IMAGE_TOPIC_4;

#endif /* M7_TRACKING_INCLUDE_M7_TRACKING_PARAMS_H_ */
