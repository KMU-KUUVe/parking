#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

#ifndef PARKING_H
#define PARKING_H

#define PI 3.141592

/**
*@brief Definition of the Parking class. It contains all the functions and variables depicted in the
*@brief Activity diagram and UML Class diagram.
*@brief It detects the lanes in an image if a highway and outputs the
*@brief same image with the plotted lane.
*/
class Parking {
private:
	double img_size;
	double img_center;

	bool p_stop = false;
	unsigned int stop_count = 0;
	bool old_value = false;
	unsigned int PIXCEL_N = 30; // number of pixcel to check in stop_detect function.
	unsigned int STOP_THRES = 200; // critria of pixcel values
	unsigned int ROW_LOCATE = 90; // detect point of row axis. the location is the percentage of top to img raw's rows.
	unsigned int COL_LOCATE = 50; // detect point of column axis. the location is the percentage of top to img raw's clos.
	int sign_goal_ = 0;
 	unsigned int LANE_ROW_LOCATE = 70;


public:
	cv::Mat deNoise(cv::Mat inputImage);  // Apply Gaussian blurring to the input Image
	cv::Mat mask(cv::Mat frame);
	double steer_control(cv::Mat denoise, int height_percent, int judging_line, cv::Mat frame, int sign_goal, int right_detect_offset, int left_detect_offset);
	int detectstoppoint(cv::Mat img_filtered_,cv::Mat _img_bgr, int stop_change_count, int detect_layer);
	bool detectlanestop(cv::Mat img_filtered_,cv::Mat _img_bgr, int stop_change_count, int detect_layer);
	bool stop_detect(cv::Mat img_filtered, int detect_layer, int row_locate);
	void VisualizeCircle(cv::Mat _img_bgr, cv::Mat _img_filtered, int detect_layer);
	void filter_colors(cv::Mat _img_bgr, cv::Mat &img_filtered);
};

#endif
