#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "parking/Parking.h"

using namespace std;
using namespace cv;

#define WHITE_THRESH 150
//���� ���� ����
Scalar lower_white = Scalar(WHITE_THRESH, WHITE_THRESH, WHITE_THRESH); //��� ���� (RGB)
Scalar upper_white = Scalar(255, 255, 255);
Scalar lower_yellow = Scalar(10, 100, 100); //����� ���� (HSV)
Scalar upper_yellow = Scalar(40, 255, 255);

//bulrring the image
cv::Mat Parking::deNoise(cv::Mat inputImage) {
  cv::Mat output;

  cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);     // ����þ� ��
//  medianBlur(inputImage, output, 3);    // �̵�� �� -> ���� ���� ������.

  return output;
}

//filter the image by yellow and white color
void Parking::filter_colors(Mat _img_bgr, Mat &img_filtered)
{
	// Filter the image to include only yellow and white pixels
	Mat img_bgr;
	_img_bgr.copyTo(img_bgr);
  Mat img_gary;

  cvtColor(img_bgr, img_gary, CV_BGR2GRAY);
  threshold( img_gary, img_filtered, THRESH_BINARY, 255, 0);

/* //  code from lane detector.
  Mat img_hsv, img_combine;
	Mat white_mask, white_image;
	Mat yellow_mask, yellow_image;

	//Filter white pixels
	inRange(img_bgr, lower_white, upper_white, white_mask);
	bitwise_and(img_bgr, img_bgr, white_image, white_mask);

	//Filter yellow pixels( Hue 30 )
	cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);

	inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
	bitwise_and(img_bgr, img_bgr, yellow_image, yellow_mask);

	//Combine the two above images
	addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, img_combine);

	img_combine.copyTo(img_filtered);
*/


}

cv::Mat Parking::mask(cv::Mat frame) {
    frame(Rect(0, 0, frame.cols , frame.rows/2))=0;
    return frame;
}

bool Parking::detectstoppoint(cv::Mat img_filtered_,cv::Mat _img_bgr)
{
  Mat img_filtered;
  img_filtered_.copyTo(img_filtered);
  Mat img_bgr;
  _img_bgr.copyTo(img_bgr);

  cout << "detect_stop_point" << endl;

  if(!p_stop){
    cout << "start detecting" << endl;

    //Compare the old and new(present) stop_detect value.
    //If the values are diffrent, it means that the detect point are on the border.
    //At that time count up the stop count value.
    bool present_value = stop_detect(img_filtered);
    if (old_value != present_value){
      stop_count++;
      old_value = present_value;
    }
    cout << "stop_count : " << stop_count << endl;

    //if stop count is 3, that means the detect point are on the end parking line.
    if(stop_count >= 3){
      p_stop = true;
      cout << "stop" << endl;
      return true;
    }
    else{
      p_stop = false;
      return false;
    }
  }
  return false;
}

bool Parking::stop_detect(cv::Mat img_filtered)
{
  Mat chk_img;
  img_filtered.copyTo(chk_img);
  for(int i=0; i < PIXCEL_N; i++){
    if(chk_img.at<uchar>(chk_img.rows * (int)ROW_LOCATE/100  , chk_img.cols * (int)COL_LOCATE / 100 - PIXCEL_N/2 + i) < STOP_THRES)
      return false;
  }
  return true;
}

void Parking::VisualizeCircle(cv::Mat _img_bgr, cv::Mat _img_filtered)
{
  Mat img_filtered;
  _img_filtered.copyTo(img_filtered);
  Mat img_bgr;
  _img_bgr.copyTo(img_bgr);

  cout << "p_stop : " << p_stop << endl;
  circle(img_bgr, Point(img_bgr.cols * (int)COL_LOCATE / 100, img_bgr.rows * (int)ROW_LOCATE/100), 5, Scalar(255, 0, 255 * p_stop), -1);
  circle(img_filtered, Point(img_filtered.cols * (int)COL_LOCATE / 100, img_filtered.rows * (int)ROW_LOCATE/100), 5, Scalar(255, 0, 255 * p_stop), -1);
  cout << "visualize" << endl;
  imshow("parking_raw",img_bgr);
  imshow("parking_filter",img_filtered);

}
