#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "parking/Parking.h"

using namespace std;
using namespace cv;


void Parking::filter_colors(Mat _img_bgr, Mat &img_filtered)
{
	Scalar lower_white_rgb = Scalar(200, 200, 200); //(RGB)
	Scalar upper_white_rgb = Scalar(255, 255, 255);
	Scalar lower_white_hsv = Scalar(0, 0, 180); //(HSV)
	Scalar upper_white_hsv = Scalar(360, 65, 255);
	
	// Filter the image to include only yellow and white pixels
	Mat img_bgr;
	_img_bgr.copyTo(img_bgr);
	Mat test;
	_img_bgr.copyTo(test);
	Mat img_hsv, img_combine;
	Mat white_mask_rgb, white_image_rgb;
	Mat white_mask_hsv, white_image_hsv;
	Mat thresh, gray;

	//Filter white pixels with RGB
	inRange(img_bgr, lower_white_rgb, upper_white_rgb, white_mask_rgb);
	bitwise_and(img_bgr, img_bgr, white_image_rgb, white_mask_rgb);

 	cvtColor(white_image_rgb,white_image_rgb,COLOR_BGR2GRAY);
	cv::threshold(white_image_rgb, white_image_rgb, 170, 255, cv::THRESH_BINARY);
	imshow("white_rgb", white_image_rgb);

	white_image_rgb.copyTo(img_filtered);
}

//bulrring the image
cv::Mat Parking::deNoise(cv::Mat inputImage) {
	cv::Mat output;
	cv::medianBlur(inputImage, output, 3);
	return output;
}

cv::Mat Parking::mask(cv::Mat frame) {
	frame(Rect(0, 0, frame.cols , frame.rows/2))=0;
	return frame;
}

int Parking::detectstoppoint(cv::Mat img_filtered_,cv::Mat _img_bgr, int stop_change_count, int detect_layer)
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
		bool present_value = stop_detect(img_filtered, detect_layer, ROW_LOCATE);
		cout << "present_value" << present_value << endl;
		if (old_value != present_value){
			stop_count++;
			old_value = present_value;
		}
		cout << "stop_count : " << stop_count << endl;

		//if stop count is 3, that means the detect point are on the end parking line.
		if(stop_count >= stop_change_count){
			p_stop = true;
			cout << "stop" << endl;
			//return true;
		}
		else{
			p_stop = false;
			//return false;
		}
	}
	return stop_count;
}

bool Parking::detectlanestop(cv::Mat img_filtered_,cv::Mat _img_bgr, int stop_change_count, int detect_layer)
{
	Mat img_filtered;
	img_filtered_.copyTo(img_filtered);
	Mat img_bgr;
	_img_bgr.copyTo(img_bgr);

	cout << "start detecting" << endl;

	//Compare the old and new(present) stop_detect value.
	//If the values are diffrent, it means that the detect point are on the border.
	//At that time count up the stop count value.
	bool is_detected = stop_detect(img_filtered, detect_layer, LANE_ROW_LOCATE);
	cout << "is_detected: " << is_detected << endl;
	if(is_detected)
	{
		cout << "lane detecting stop" << endl;
		return true;
	}
	else{
		return false;
	}
}

bool Parking::stop_detect(cv::Mat img_filtered, int detect_layer, int row_locate)
{
	Mat chk_img;
	img_filtered.copyTo(chk_img);

	cout << "in the function" << endl;

	for(int j = detect_layer-1; j >= 0; j--){
		for(int i = 0; i < PIXCEL_N; i++){
			if(chk_img.at<uchar>(chk_img.rows * (int)row_locate/100 - j*10  , chk_img.cols * (int)COL_LOCATE / 100 - PIXCEL_N/2 + i) < STOP_THRES){
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
		circle(img_bgr, Point(img_bgr.cols * (int)(COL_LOCATE) / 100, img_bgr.rows * (int)ROW_LOCATE/100 - j*10), 5, Scalar(255, 0, 255 * p_stop), -1);
		circle(img_filtered, Point(img_filtered.cols * (int)(COL_LOCATE) / 100, img_filtered.rows * (int)ROW_LOCATE/100 -j*10), 5, Scalar(255, 0, 255 * p_stop), -1);
		//circle(img_bgr, Point(img_bgr.cols * (int)(COL_LOCATE +20) / 100, img_bgr.rows * (int)ROW_LOCATE/100 - j*10), 5, Scalar(255, 0, 255 * p_stop), -1);
		//circle(img_filtered, Point(img_filtered.cols * (int)(COL_LOCATE +20) / 100, img_filtered.rows * (int)ROW_LOCATE/100 -j*10), 5, Scalar(255, 0, 255 * p_stop), -1);
	}
	//cout << "visualize" << endl;
	imshow("parking_raw",img_bgr);
	imshow("parking_filter",img_filtered);
	waitKey(3);
}

double Parking::steer_control(Mat img, int height_percent, int judging_line, Mat frame, int sign_goal, int right_detect_offset, int left_detect_offset)
{
	Mat denoise;
	img.copyTo(denoise);

 sign_goal_ = sign_goal;
 int left_x_num = 0;
 int right_x_num = 0;

 int left_sum_x = 0;
 int right_sum_x = 0;

 int left_x = 0;
 int right_x = denoise.cols;

 int line_height = denoise.rows * height_percent / 100.0;

 for (int j = line_height ; j < line_height + judging_line; j++)
 {
	 for (int i = 2; i < denoise.cols / 2.5; i++)
	 {

		 //left lane.
		 if (denoise.at<uchar>(j, i) == 255)
		 {
			 left_sum_x += i;
			 left_x_num++;

		 }

		 //right lane
		 if (denoise.at<uchar>(j, denoise.cols - i) == 255)
		 {
			 right_sum_x += denoise.cols - i;
			 right_x_num++;
		 }
	 }
 }

 line(denoise, Point(2, line_height), Point(denoise.cols, line_height), Scalar(0, 255, 255), 5);
 line(denoise, Point(denoise.cols - 2, line_height), Point(denoise.cols * 2 / 2.5, line_height), Scalar(255, 255, 255), 5);

 if (left_x_num > 20)
 {
	 left_x = left_sum_x / left_x_num;
 }
 if (right_x_num > 20)
 {
	 right_x = right_sum_x / right_x_num;
 }


 int middle = (left_x + right_x) / 2.0;
double angle = 0;

if(sign_goal_ == 1){
	angle = atan2(middle+right_detect_offset - denoise.cols / 2, denoise.rows - line_height) * 180 / PI;
}else if(sign_goal_ == 2){
	angle = atan2(middle-left_detect_offset - denoise.cols / 2, denoise.rows - line_height) * 180 / PI;
}


 // plot
 line(frame, Point(2, line_height), Point(frame.cols / 3.0, line_height), Scalar(0, 255, 255), 5);
 line(frame, Point(frame.cols - 2, line_height), Point(frame.cols * 2 / 2.5, line_height), Scalar(255, 255, 255), 5);
 //circle(frame, Point(frame.cols/2, frame.rows/2), 5, Scalar(255, 0, 0), 5);
 circle(frame, Point(left_x, line_height), 5, Scalar(255, 0, 0), 5);
 circle(frame, Point(right_x, line_height), 5, Scalar(255, 0, 0), 5);

	if(sign_goal_ == 1){
		line(frame, Point(middle+right_detect_offset, line_height), Point(denoise.cols / 2, denoise.rows), Scalar(255, 255, 255), 4);
	}else if(sign_goal_ == 2){
		 line(frame, Point(middle-left_detect_offset, line_height), Point(denoise.cols / 2, denoise.rows), Scalar(255, 255, 255), 4);
	}

 
 imshow("frame", frame);
waitKey(3);
return angle;
}
