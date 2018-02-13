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
#define	D_RED_HUE_HSV_LOW 0
#define	D_RED_SATURATION_HSV_HIGH 256
#define	D_RED_SATURATION_HSV_LOW 0
#define	D_RED_VALUE_HSV_HIGH 256
#define	D_RED_VALUE_HSV_LOW 0


#define D_GREEN_HUE_HSV_HIGH 256
#define D_GREEN_HUE_HSV_LOW 0
#define D_GREEN_SATURATION_HSV_HIGH 256
#define D_GREEN_SATURATION_HSV_LOW 0
#define D_GREEN_VALUE_HSV_HIGH 256
#define D_GREEN_VALUE_HSV_LOW 0

#define D_CANNY_THRESHOLD 100

extern int GREEN_HUE_HSV_HIGH;
extern int GREEN_HUE_HSV_LOW;
extern int GREEN_SATURATION_HSV_HIGH;
extern int GREEN_SATURATION_HSV_LOW;
extern int GREEN_VALUE_HSV_HIGH;
extern int GREEN_VALUE_HSV_LOW;

extern int RED_HUE_HSV_HIGH;
extern int RED_HUE_HSV_LOW;
extern int RED_SATURATION_HSV_HIGH;
extern int RED_SATURATION_HSV_LOW;
extern int RED_VALUE_HSV_HIGH;
extern int RED_VALUE_HSV_LOW;

extern int CANNY_THRESHOLD;


extern std::string CAMERA_IMAGE_TOPIC_1;

extern std::string CAMERA_IMAGE_TOPIC_2;

extern std::string CAMERA_IMAGE_TOPIC_3;

extern std::string CAMERA_IMAGE_TOPIC_4;

#endif /* M7_TRACKING_INCLUDE_M7_TRACKING_PARAMS_H_ */
