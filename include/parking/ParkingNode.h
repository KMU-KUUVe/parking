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
#include "state_cpp_msg/MissionPlannerAction.h"
#include <actionlib/server/simple_action_server.h>

#ifndef PARKINGNODE_H
#define PARKINGNODE_H

#define resize_n 1

class ParkingNode
{
public:
    ParkingNode();
    ParkingNode(cv::String path);
    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    void actionCallback(const state_cpp_msg::MissionPlannerGoalConstPtr& goal);
    int laneDetecting();
    void parkingdetect_A();
    void parkingdetect_B();
    int parkingstart();

protected:
	void getRosParamForUpdate();
	void parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img);
  ackermann_msgs::AckermannDriveStamped makeControlMsg();

protected:
  ros::NodeHandle nh_;
  ros::Publisher control_pub_;	// Controll 메시지를 Publish하는 Publisher
  ros::Subscriber image_sub_;		// 가공되지 않은 raw image 메시지를 Subscribe하는 Subscriber
  actionlib::SimpleActionServer<state_cpp_msg::MissionPlannerAction> as_;
  bool mission_start = false;
  bool mission_cleared = false;
  int sign_goal = 0;
  bool change_lane = true;
  int CONST_THROTTLE = 4;
  int throttle_ = 4;
  int steer_control_value_= 0;

  LaneDetector lanedetector; //Create the class object
	Parking parking;  // Create the class object

  // image preprocessing
  cv::Mat frame;
  cv::Mat lane_frame;
  cv::Mat parking_frame;
  cv::Mat img_denoise;
  cv::Mat img_mask;
  cv::Mat img_mask2;

  //steer comtrol
	int steer_height = 70;
	int i = 0;
	double avg = 0;
	double sum = 0;
	int frame_count = 0;
	int j = 0;
	double angle = 0;
	int left_x = 0;
	int right_x = 0;
  double angle_factor_ = 1.0;

  //parking
  bool parking_stop = false;

	cv::String test_video_path = "";
};

#endif
