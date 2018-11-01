#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

#ifndef PARKING_H
#define PARKING_H

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
	unsigned int PIXCEL_N = 6; // number of pixcel to check in stop_detect function.
	unsigned int STOP_THRES = 200; // critria of pixcel values
	unsigned int ROW_LOCATE = 80; // detect point of row axis. the location is the percentage of top to img raw's rows.
	unsigned int COL_LOCATE = 50; // detect point of column axis. the location is the percentage of top to img raw's clos.
	unsigned int THRESH_BINARY = 150; // binary threshold
	unsigned int WHITE_THRESH = 150; // white color threshold
	unsigned int YELLOW_THRESH; // yeoow color threshold
/*
	Scalar lower_white; //��� ���� (RGB)
	Scalar upper_white;
	Scalar lower_yellow; //����� ���� (HSV)
	Scalar upper_yellow;
*/
public:
	cv::Mat deNoise(cv::Mat inputImage);  // Apply Gaussian blurring to the input Image
	void filter_colors(cv::Mat _img_bgr, cv::Mat &img_filtered);
	cv::Mat mask(cv::Mat frame);
	bool detectstoppoint(cv::Mat img_filtered_, cv::Mat _img_bgr);
	bool stop_detect(cv::Mat img_filtered);
	void VisualizeCircle(cv::Mat _img_bgr, cv::Mat _img_filtered);


};

#endif
