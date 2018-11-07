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
	//regression variables
	double detect_n = 0.30; // detection point(line) of y axis for line regression(also apply to visualization).(the percentage of image column)
	//uchar steer_height = 70; //decide line_middle (line_middle.y = steer_height / 100.0 * inputImage.rows)
	//bool left_flag = false;  // Tells us if there's left boundary of lane detected
	//bool right_flag = false;  // Tells us if there's right boundary of lane detected
	cv::Point right_b;  // Members of both line equations of the lane boundaries:
	//double right_m;  // y = m*x + b
	cv::Point left_b;  //
	//double left_m;  //

	bool p_stop = false;
	unsigned int stop_count = 0;
	bool old_value = false;
	unsigned int PIXCEL_N = 16; // number of pixcel to check in stop_detect function.
	unsigned int STOP_THRES = 200; // critria of pixcel values
	unsigned int ROW_LOCATE = 95; // detect point of row axis. the location is the percentage of top to img raw's rows.
	unsigned int COL_LOCATE = 50; // detect point of column axis. the location is the percentage of top to img raw's clos.


public:
	cv::Mat deNoise(cv::Mat inputImage);  // Apply Gaussian blurring to the input Image
	cv::Mat mask(cv::Mat frame);
	double steer_control(cv::Mat denoise, int height_percent, int judging_line, int &left_x, int &right_x , cv::Mat frame);

	bool detectstoppoint(cv::Mat img_filtered_,cv::Mat _img_bgr, int stop_change_count, int detect_layer);
	bool stop_detect(cv::Mat img_filtered, int detect_layer);
	void VisualizeCircle(cv::Mat _img_bgr, cv::Mat _img_filtered, int detect_layer);
	std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage, double &angle);

};

#endif
