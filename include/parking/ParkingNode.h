#include "ros/ros.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "parking/Parking.h"
#include "lane_detector/LaneDetector.h"
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <signal.h>

#ifndef PARKINGNODE_H
#define PARKINGNODE_H

#define resize_n 1

class ParkingNode
{
public:
    ParkingNode();

    ParkingNode(cv::String path);


    //Run test that use video file.
    bool run_test();

    /**
	 * @brief 카메라로부터 들어온 이미지를 Subscribe 했을 때 호출되는 Callback 함수
	 *
	 * @param image 카메라 드라이버 노드에서 보낸 이미지를 받아 포인팅하고있는 포인터
	 *
	 */
    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    int laneDetecting();
    void parkingdetect();
    bool parkingstart();

protected:
    /**
	 * @brief 차선 인식과 관련된 파라미터 중 동적으로 바뀔 수 있는 값들을 읽어오는 함수
	 *
	 * @details 이 함수는 주기적으로 계속 호출되므로, rosparam을 통해 노드 실행중 동적으로 값들을 바꾸면서 테스트가 가능하다.
	 */
	void getRosParamForUpdate();

	/**
	 * @brief Ros 통신에서 사용하는 이미지 타입을 Opencv의 Mat 타입으로 변환해주는 함수
	 *
	 */
	void parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img);

	/**
	 * @brief make control message
	 *
	 */
    ackermann_msgs::AckermannDriveStamped makeControlMsg();

    /**
     * @brief lane detecting wrapper
     *
     */


protected:
  ros::NodeHandle nh_;
	ros::Publisher control_pub_;	// Controll 메시지를 Publish하는 Publisher
	ros::Subscriber image_sub_;		// 가공되지 않은 raw image 메시지를 Subscribe하는 Subscriber


  LaneDetector lanedetector;
	Parking parking;  // Create the class object
	cv::Mat frame;
  cv::Mat img_denoise;
  cv::Mat img_edges;
  cv::Mat img_mask;
  cv::Mat img_lines;
  cv::Mat img_mask2;
  cv::Mat img_mask3;
  int Mask_method = 1;
  std::vector<cv::Vec4i> lines;
  std::vector<std::vector<cv::Vec4i> > left_right_lines;
  std::vector<cv::Point> lane;
  std::string turn;
	int flag_plot = -1;
	int i = 0;
	double avg = 0;
	double sum = 0;
	int frame_count = 0;
	int j = 0;
	double angle = 0;
  bool parking_stop = false;
  int throttle_ = 5;
  int steer_control_value_= 0;
  double angle_factor_ = 1.0;
	cv::String test_video_path = "";
};

#endif
