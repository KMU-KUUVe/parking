#include "parking/ParkingNode.h"

using namespace std;
using namespace cv;

ParkingNode::ParkingNode()
	:as_(nh_, "parking", boost::bind(&ParkingNode::actionCallback, this, _1), false)
{
	nh_ = ros::NodeHandle("~");
	as_.start();

	/* if NodeHangle("~"), then (write -> /parking/write)	*/
	control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);

	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &ParkingNode::imageCallback, this);

	int resize_width = 0;
	int resize_height = 0;
	int steer_max_angle = 0;
	int detect_line_count = 0;
	int stop_distance = 0;
	int stop_time = 0;
	getRosParamForUpdate();

}

void ParkingNode::actionCallback(const state_cpp_msg::MissionPlannerGoalConstPtr& goal)
{
	cout << "parking actioniCallback called" << endl;
	sign_goal = goal->mission;
	ros::Rate r(10);
	while(ros::ok()){
		if(change_lane){
			if(sign_goal == 1){
				parkingdetect_A();
			}
			else if(sign_goal == 2){
				parkingdetect_B();
			}
		}
		else{
			mission_start = true;
			if(mission_cleared){
				state_cpp_msg::MissionPlannerResult result;
				as_.setSucceeded(result);
				mission_start = false;
				break;
			}
		}
		r.sleep();
	}
}


void ParkingNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	if(mission_start){
		try{
			parseRawimg(image, frame);
		} catch(const cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return ;
		} catch(const std::runtime_error& e) {
			cerr << e.what() << endl;
		}

		if(lane_detect){
			cout << "do lane detecting" << endl;
			steer_control_value_ = laneDetecting();
			if(parking.detectlanestop(img_denoise, frame, 1, 1)) {
				parking_stop = false;
				lane_detect = false;
				mission_cleared = false;
			}
		}
		else {
			int parking_count = parkingstart();


			if(parking_count < 3){
				cout << "stop lane detecting and start parking" << endl;
				parking_stop = false;
				mission_cleared = false;
			}
			else{
				cout << "stop" << endl;
				steer_control_value_ = 0;
				throttle_ = 0;
				parking_stop = true;
				mission_cleared = true;
				destroyAllWindows();
			}
		}

	cout << "throttle : " << throttle_ << "steer : " << steer_control_value_ << endl;
	ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();

	control_pub_.publish(control_msg);
	}
}


void ParkingNode::getRosParamForUpdate()
{
	nh_.getParam("throttle", throttle_);
	nh_.getParam("angle_factor", angle_factor_);
}


ackermann_msgs::AckermannDriveStamped ParkingNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	control_msg.drive.steering_angle = steer_control_value_;
	control_msg.drive.speed = throttle_;
	return control_msg;
}

int ParkingNode::laneDetecting()
{
	int ncols = frame.cols;
	int nrows = frame.rows;
	double angle_ =  0;


	resize(frame, lane_frame, Size(ncols / resize_n, nrows / resize_n));
	img_mask = lanedetector.mask(lane_frame);
	//imshow("img_mask", img_mask);
	lanedetector.filter_colors(img_mask, img_mask2);
	//imshow("img_mask2", img_mask2);
	img_denoise = lanedetector.deNoise(img_mask2);
	//imshow("img_denoise", img_denoise);
	/*indoor test*/
	//bitwise_not(img_denoise,img_denoise);

		double angle = parking.steer_control(img_denoise, steer_height, 5, img_mask, sign_goal, 40, 60);

		waitKey(3);

		angle_ = angle * angle_factor_;
		if(angle_ > 23){
			angle_ = 23;
		}
		else if(angle_ < -23){
			angle_ = -23;
		}

		return angle_;
}


int ParkingNode::parkingstart()
{
	int throttle;
	int ncols = frame.cols;
	int nrows = frame.rows;

	resize(frame, lane_frame, Size(ncols / resize_n, nrows / resize_n));
	img_mask = lanedetector.mask(lane_frame);
	//imshow("img_mask", img_mask);
	parking.filter_colors(img_mask, img_mask2);
	//imshow("img_mask2", img_mask2);
	img_denoise = lanedetector.deNoise(img_mask2);
	//imshow("img_denoise", img_denoise);
	/*indoor test*/
	//bitwise_not(img_denoise,img_denoise);

	double angle = parking.steer_control(img_denoise, steer_height, 5, img_mask, sign_goal, 0, 0);

	angle = angle * angle_factor_;
	if(angle > 23){
		angle = 23;
	}
	else if(angle < -23){
		angle = -23;
	}

	steer_control_value_ = angle;

	parking.VisualizeCircle(frame, img_denoise, 1);

	return parking.detectstoppoint(img_denoise, frame, 3, 1);

}

void ParkingNode::parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);

	cv_img = cv_ptr->image;

	if (cv_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}
}


void ParkingNode::parkingdetect_A()
{
	cout << "parking A" << endl;
	throttle_ = 0;
	steer_control_value_ = 0;
	ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();
	control_pub_.publish(control_msg);
	cout << "throttle : " << throttle_ << "steer : " << steer_control_value_ << endl;
	ros::Duration(1).sleep();
	//getRosParamForUpdate();
	throttle_ = 8;
	steer_control_value_ = -25;
	control_msg = makeControlMsg();
	control_pub_.publish(control_msg);
	cout << "throttle : " << throttle_ << "steer : " << steer_control_value_ << endl;
	ros::Duration(5.3).sleep();
	throttle_ = 7;
	steer_control_value_ = 20;
	control_msg = makeControlMsg();
	control_pub_.publish(control_msg);
	cout << "throttle : " << throttle_ << "steer : " << steer_control_value_ << endl;
	ros::Duration(2.7).sleep();
	throttle_ = CONST_THROTTLE;
	steer_control_value_ = 0;
	control_msg = makeControlMsg();
	control_pub_.publish(control_msg);
	cout << "throttle : " << throttle_ << "steer : " << steer_control_value_ << endl;
	change_lane = false;
}

void ParkingNode::parkingdetect_B()
{
	cout << "parking B" << endl;
	throttle_ = 0;
	steer_control_value_ = 0;
	ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();
	control_pub_.publish(control_msg);
	cout << "throttle : " << throttle_ << "steer : " << steer_control_value_ << endl;
	ros::Duration(1).sleep();
	//getRosParamForUpdate();
	throttle_ = 8;
	steer_control_value_ = 22;
	control_msg = makeControlMsg();
	control_pub_.publish(control_msg);
	cout << "throttle : " << throttle_ << "steer : " << steer_control_value_ << endl;
	ros::Duration(4.3).sleep();
	throttle_ = 7;
	steer_control_value_ = -22;
	control_msg = makeControlMsg();
	control_pub_.publish(control_msg);
	cout << "throttle : " << throttle_ << "steer : " << steer_control_value_ << endl;
	ros::Duration(2.6).sleep();
	throttle_ = CONST_THROTTLE;
	steer_control_value_ = 0;
	control_msg = makeControlMsg();
	control_pub_.publish(control_msg);
	cout << "throttle : " << throttle_ << "steer : " << steer_control_value_ << endl;
	change_lane = false;
}
