#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "parking/Parking.h"

using namespace std;
using namespace cv;

//ȭ�� resize

#define resize_n 2
#define LINE_LENGTH 30

// ���Ⱒ �Ǵ� ��ġ y
#define steer_height 70

#define PI 3.141592

//���� ���� ����
Scalar lower_white = Scalar(200, 200, 200); //��� ���� (RGB)
Scalar upper_white = Scalar(255, 255, 255);
Scalar lower_yellow = Scalar(10, 100, 100); //����� ���� (HSV)
Scalar upper_yellow = Scalar(40, 255, 255);




// ����þ� ��, �ŵ�� ���� ���
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

	  // �ð�������� ����Ʈ�� �����Ѵ�.
	  // Point(x,y)
	  // TODO : �Ʒ� 4���� ������ Ư�� ���� �ƴ� ������ ����
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
  else if (method == 1)  //  ȭ���� ���� �ڸ���.
  {
	  return frame(Rect(0, frame.rows / 2, frame.cols , frame.rows / 2));
  }
}

bool Parking::detectstoppoint()
{
  if (stop_detect()) {
    cout << "stop point detected!" << endl;
    VisualizeCircle();
    return true;
  }
  else{
    return false;
  }
}

bool Parking::stop_detect()
{

  
}
