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
double Parking::steer_control(Mat denoise, int height_percent, int judging_line, int &left_x, int &right_x , Mat frame)
{


 int left_x_num = 0;
 int right_x_num = 0;

 int left_sum_x = 0;
 int right_sum_x = 0;

 int line_height = denoise.rows * height_percent / 100.0;

 for (int j = line_height ; j < line_height + judging_line; j++)
 {
	 for (int i = 2; i < denoise.cols / 3.0; i++)
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

 //imshow("frame", frame);
 line(denoise, Point(2, line_height), Point(denoise.cols, line_height), Scalar(0, 255, 255), 5);
 line(denoise, Point(denoise.cols - 2, line_height), Point(denoise.cols * 2 / 3.0, line_height), Scalar(255, 255, 255), 5);

 if (!(left_x_num == 0))
 {
	 left_x = left_sum_x / left_x_num;
 }
 if (!(right_x_num == 0))
 {
	 right_x = right_sum_x / right_x_num;
 }


 int middle = (left_x + right_x) / 2.0;



 double angle = atan2(middle+40 - denoise.cols / 2, denoise.rows - line_height) * 180 / PI;
 if(angle > 23){
	 angle = 23;
 }
 else if(angle < -23){
	 angle = -23;
 }



 // plot
 line(frame, Point(2, line_height), Point(frame.cols / 3.0, line_height), Scalar(0, 255, 255), 5);
 line(frame, Point(frame.cols - 2, line_height), Point(frame.cols * 2 / 3.0, line_height), Scalar(255, 255, 255), 5);
 //circle(frame, Point(frame.cols/2, frame.rows/2), 5, Scalar(255, 0, 0), 5);
 circle(frame, Point(left_x, line_height), 5, Scalar(255, 0, 0), 5);
 circle(frame, Point(right_x, line_height), 5, Scalar(255, 0, 0), 5);

 line(frame, Point(middle, line_height), Point(denoise.cols / 2, denoise.rows), Scalar(255, 255, 255), 4);
 imshow("frame", frame);
 return angle;
}
