#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "parking/Parking.h"

using namespace std;
using namespace cv;


//bulrring the image
cv::Mat Parking::deNoise(cv::Mat inputImage) {
	cv::Mat output;

	//cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);
	cv::medianBlur(inputImage, output, 3);

	return output;
}

cv::Mat Parking::mask(cv::Mat frame) {
	frame(Rect(0, 0, frame.cols , frame.rows/2))=0;
	return frame;
}

bool Parking::detectstoppoint(cv::Mat img_filtered_,cv::Mat _img_bgr, int stop_change_count, int detect_layer)
{
	Mat img_filtered;
	img_filtered_.copyTo(img_filtered);
	Mat img_bgr;
	_img_bgr.copyTo(img_bgr);

	if(!p_stop){
		cout << "start detecting" << endl;

		//Compare the old and new(present) stop_detect value.
		//If the values are diffrent, it means that the detect point are on the border.
		//At that time count up the stop count value.
		bool present_value = stop_detect(img_filtered, detect_layer);
		if (old_value != present_value){
			stop_count++;
			old_value = present_value;
		}
		cout << "stop_count : " << stop_count << endl;

		//if stop count is 3, that means the detect point are on the end parking line.
		if(stop_count >= stop_change_count){
			p_stop = true;
			cout << "stop" << endl;
			return true;
		}
		else{
			p_stop = false;
			return false;
		}
	}
	return true;
}

bool Parking::stop_detect(cv::Mat img_filtered, int detect_layer)
{
	Mat chk_img;
	img_filtered.copyTo(chk_img);

	cout << "in the function" << endl;

	for(int j = detect_layer-1; j >= 0; j--){
		for(int i = 0; i < PIXCEL_N; i++){
			//cout << chk_img.at<uchar>(chk_img.rows * (int)ROW_LOCATE/100 - j*10  , chk_img.cols * (int)COL_LOCATE / 100 - PIXCEL_N/2 + i) << endl;
			if((chk_img.at<uchar>(chk_img.rows * (int)ROW_LOCATE/100 - j*10  , chk_img.cols * (int)(COL_LOCATE-20) / 100 - PIXCEL_N/2 + i) < STOP_THRES) && (chk_img.at<uchar>(chk_img.rows * (int)ROW_LOCATE/100 - j*10  , chk_img.cols * (int)(COL_LOCATE+20) / 100 - PIXCEL_N/2 + i) < STOP_THRES)){
				return false;
			}
		}
	}
	return true;
}

void Parking::VisualizeCircle(cv::Mat _img_bgr, cv::Mat _img_filtered, int detect_layer)
{
	Mat img_filtered;
	_img_filtered.copyTo(img_filtered);
	Mat img_bgr;
	_img_bgr.copyTo(img_bgr);

	cout << "p_stop : " << p_stop << endl;
	for(int j = detect_layer -1; j >= 0; j--){
		circle(img_bgr, Point(img_bgr.cols * (int)(COL_LOCATE -20) / 100, img_bgr.rows * (int)ROW_LOCATE/100 - j*10), 5, Scalar(255, 0, 255 * p_stop), -1);
		circle(img_filtered, Point(img_filtered.cols * (int)(COL_LOCATE -20) / 100, img_filtered.rows * (int)ROW_LOCATE/100 -j*10), 5, Scalar(255, 0, 255 * p_stop), -1);
		circle(img_bgr, Point(img_bgr.cols * (int)(COL_LOCATE +20) / 100, img_bgr.rows * (int)ROW_LOCATE/100 - j*10), 5, Scalar(255, 0, 255 * p_stop), -1);
		circle(img_filtered, Point(img_filtered.cols * (int)(COL_LOCATE +20) / 100, img_filtered.rows * (int)ROW_LOCATE/100 -j*10), 5, Scalar(255, 0, 255 * p_stop), -1);
	}
	//cout << "visualize" << endl;
	imshow("parking_raw",img_bgr);
	imshow("parking_filter",img_filtered);
}

std::vector<cv::Point> Parking::regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage, double &angle) {
	std::vector<cv::Point> output(4);
	cv::Point ini;
	cv::Point fini;
	cv::Point ini2;
	cv::Point fini2;
	cv::Vec4d right_line;
	cv::Vec4d left_line;
	Point line_middle;
	std::vector<cv::Point> right_pts;
	std::vector<cv::Point> left_pts;
	int ini_y = inputImage.rows;
	int fin_y = inputImage.rows * detect_n ;
	double right_ini_x;
	double right_fin_x;
	double right_xx;
	double left_ini_x;
	double left_fin_x;
	double left_xx;

	line_middle.y = steer_height / 100.0 * inputImage.rows;

	// If right lines are being detected, fit a line using all the init and final points of the lines

	if (right_flag == true) {
		for (auto i : left_right_lines[0]) {
			ini = cv::Point(i[0], i[1]);
			fini = cv::Point(i[2], i[3]);

			right_pts.push_back(ini);
			right_pts.push_back(fini);
		}

		if (right_pts.size() > 0) {
			// The right line is formed here
			cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
			right_m = right_line[1] / right_line[0];
			right_b = cv::Point(right_line[2], right_line[3]);
		}
		right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
		right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;
		right_xx = (line_middle.y - right_b.y) / (right_m)+right_b.x;


	}
	else{
		right_ini_x = inputImage.cols;
		right_fin_x = inputImage.cols;
		right_xx = inputImage.cols;


	}

	// If left lines are being detected, fit a line using all the init and final points of the lines
	if (left_flag == true) {
		for (auto j : left_right_lines[1]) {
			ini2 = cv::Point(j[0], j[1]);
			fini2 = cv::Point(j[2], j[3]);

			left_pts.push_back(ini2);
			left_pts.push_back(fini2);
		}

		if (left_pts.size() > 0) {
			// The left line is formed here
			cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
			left_m = left_line[1] / left_line[0];
			left_b = cv::Point(left_line[2], left_line[3]);
		}
		left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
		left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;
		left_xx = (line_middle.y - left_b.y) / (left_m)+left_b.x;


	}
	else{
		left_ini_x = 0;
		left_fin_x = 0;
		left_xx = 0;
	}

	// One the slope and offset points have been obtained, apply the line equation to obtain the line points
	line_middle.x = (right_xx + left_xx)/2.0;

	circle(inputImage, line_middle , 5, Scalar(255, 255, 0), 5);

	angle = atan2((line_middle.x - 30) - inputImage.cols / 2.0, inputImage.rows - line_middle.y) * 180 / PI;
	//angle = -atan2(line_middle.x - inputImage.cols / 2.0, inputImage.rows - line_middle.y) * 180 / PI;

	if(angle >23){
		angle = 23;
	}else if(angle < -23){
		angle = -23;
	}
	//

	output[0] = cv::Point(right_ini_x, ini_y);
	output[1] = cv::Point(right_fin_x, fin_y);
	output[2] = cv::Point(left_ini_x, ini_y);
	output[3] = cv::Point(left_fin_x, fin_y);

	return output;
}
