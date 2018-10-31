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
	//bool parking_stop_ = false;


public:
	cv::Mat deNoise(cv::Mat inputImage);  // Apply Gaussian blurring to the input Image
	void filter_colors(cv::Mat _img_bgr, cv::Mat &img_filtered);
	cv::Mat mask(cv::Mat frame, int method);
	bool detectstoppoint(cv::Mat img_filtered_, cv::Mat _img_bgr);
	bool stop_detect(cv::Mat img_filtered);
	void VisualizeCircle(cv::Mat _img_bgr, cv::Mat _img_filtered);


};

#endif
