#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "parking/Parking.h"

using namespace std;
using namespace cv;

//ȭ�� resize

#define resize_n 2
#define steer_height 70
#define PI 3.141592
#define STOP_DISTANCE 40
#define STOP_THRES 200
#define ROW_LOCATE 50
#define COL_LOCATE 80

//���� ���� ����
Scalar lower_white = Scalar(200, 200, 200); //��� ���� (RGB)
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
}

cv::Mat Parking::mask(cv::Mat frame, int method) {
  cv::Mat output;

  // ���� ���ϴ� ����ũ�� �����Ѵ�.
  if (method == 0)
  {
	  cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());

	  // Point(x,y)
	  // TODO :
	  /*
	  cv::Point pts[4] = {
		  cv::Point(210/ resize_n, 720/ resize_n),
		  cv::Point(550/ resize_n, 450/ resize_n),
		  cv::Point(716/ resize_n, 450/ resize_n),
		  cv::Point(1280/ resize_n, 720/ resize_n)
	  };
	 */

	  cv::Point pts[4] = {
		  cv::Point(0, frame.rows),
		  cv::Point(0, frame.rows/2),
		  cv::Point(frame.cols, frame.rows / 2),
		  cv::Point(frame.cols, frame.rows)
	  };
	  // Create a binary polygon mask
	  cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
	  // Multiply the edges image and the mask to get the output
	  cv::bitwise_and(frame, mask, output);

	  return output;
  }
  else if (method == 1)
  {
	  return frame(Rect(0, frame.rows / 2, frame.cols , frame.rows / 2));
  }
}

bool Parking::detectstoppoint(cv::Mat img_filtered_,cv::Mat _img_bgr)
{
  //bool old = old_value;
  Mat img_filtered;
  img_filtered_.copyTo(img_filtered);
  Mat img_bgr;
  _img_bgr.copyTo(img_bgr);
  //imshow("fiter",img_filtered);
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
      //VisualizeCircle(img_bgr, img_filtered);
      return true;
    }
    else{
      p_stop = false;
      return false;
    }
    //VisualizeCircle(img_bgr, img_filtered);
  }
  return false;
}

bool Parking::stop_detect(cv::Mat img_filtered)
{
  //uchar chk;
  Mat chk_img;
  img_filtered.copyTo(chk_img);
  //chk = img_filtered.at<uchar>(chk_img.rows * (int)STOP_DISTANCE/100, chk_img.cols * 3 / 8);
  //cout << chk << endl;
  return chk_img.at<uchar>(chk_img.rows * (int)ROW_LOCATE/100 , chk_img.cols * (int)COL_LOCATE / 100) >= STOP_THRES;
}
void Parking::VisualizeCircle(cv::Mat _img_bgr, cv::Mat _img_filtered)
{
  Mat img_filtered;
  _img_filtered.copyTo(img_filtered);
  Mat img_bgr;
  _img_bgr.copyTo(img_bgr);
  cout << "p_stop : " << p_stop << endl;
  circle(img_bgr, Point(img_filtered.rows * (int)ROW_LOCATE/100 , img_filtered.cols * (int)COL_LOCATE / 100) , 10, Scalar(255, 0, 255 * p_stop), -1);
  cout << "visualize" << endl;
  //imshow("parking",img_bgr);
  //waitKey(3);

}
