#include "parking/ParkingNode.h"

using namespace std;
using namespace cv;

ParkingNode::ParkingNode()
{
nh_ = ros::NodeHandle("~");

/* if NodeHangle("~"), then (write -> /parking/write)	*/
control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);

image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &ParkingNode::imageCallback, this);

int resize_width = 0;
int resize_height = 0;
int steer_max_angle = 0;
int detect_line_count = 0;
int stop_distance = 0;
int stop_time = 0;
bool parking_stop_ = false;

getRosParamForUpdate();

}


ParkingNode::ParkingNode(String path)
    : test_video_path(path)
{}


void ParkingNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	try{
		parseRawimg(image, frame);
	} catch(const cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return ;
	} catch(const std::runtime_error& e) {
		cerr << e.what() << endl;
	}
    getRosParamForUpdate();


    //steer_control_value_ = laneDetecting();
    throttle_ = parkingstart();

	ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();

	control_pub_.publish(control_msg);
}


void ParkingNode::getRosParamForUpdate()
{
    nh_.getParam("throttle", throttle_);
    nh_.getParam("angle_factor", angle_factor_);
}


ackermann_msgs::AckermannDriveStamped ParkingNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	//control_msg.drive.steering_angle = steer_control_value;
	control_msg.drive.steering_angle = steer_control_value_;
	control_msg.drive.speed = throttle_;
	return control_msg;
}


int ParkingNode::parkingstart()
{
    //do lane_detecting until stop
    int throttle;
		int ncols = frame.cols;
		int nrows = frame.rows;


		int64 t1 = getTickCount();
		frame_count++;

		resize(frame, frame, Size(ncols / resize_n, nrows / resize_n));
		img_denoise = parking.deNoise(frame);
		parking.filter_colors(img_denoise, img_mask2);
    img_mask = parking.mask(img_mask2,Mask_method);
    imshow("img_mask", img_mask);

    if(!parking_stop && Parking::detectstoppoint(img_mask, frame)){
      throttle_ = 0;
      parking_stop = true;
    }

		int64 t2 = getTickCount();
		double ms = (t2 - t1) * 1000 / getTickFrequency();
		sum += ms;
		avg = sum / (double)frame_count;
		//cout << "it took :  " << ms << "ms." << "average_time : " << avg << " frame per second (fps) : " << 1000 / avg << endl;
		waitKey(3);
		ROS_INFO("it took : %6.2f [ms].  average_time : %6.2f [ms].  frame per second (fps) : %6.2f [frame/s].   steer angle : %5.2f [deg]\n", ms, avg, 1000 / avg , angle);

        return throttle;
}



void ParkingNode::parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);

	cv_img = cv_ptr->image;

	if (cv_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}
}


bool ParkingNode::run_test()
{
    if(test_video_path.empty())
    {
        ROS_ERROR("Test is failed. video path is empty! you should set video path by constructor argument");
        return false;
    }

	VideoCapture cap;
	//cap.open("../../kasa.mp4");
	cap.open(test_video_path);

	if (!cap.isOpened())
    {
        ROS_ERROR("Test is failed. video is empty! you should check video path (constructor argument is correct)");
        return false;
    }

	while (1) {
		// Capture frame
		if (!cap.read(frame))
			break;


		int ncols = frame.cols;
		int nrows = frame.rows;


		int64 t1 = getTickCount();
		frame_count++;

		resize(frame, frame, Size(ncols / resize_n, nrows / resize_n));

		img_denoise = parking.deNoise(frame);


		parking.filter_colors(img_denoise, img_mask2);
    img_mask = parking.mask(img_mask2,Mask_method);
    imshow("img_mask", img_mask);


		int64 t2 = getTickCount();
		double ms = (t2 - t1) * 1000 / getTickFrequency();
		sum += ms;
		avg = sum / (double)frame_count;
		waitKey(25);
		//cout << "it took :  " << ms << "ms." << "average_time : " << avg << " frame per second (fps) : " << 1000 / avg << endl;

		printf("it took : %6.2f [ms].  average_time : %6.2f [ms].  frame per second (fps) : %6.2f [frame/s].   steer angle : %5.2f [deg]\n", ms, avg, 1000 / avg , angle);
	}

}

void ParkingNode::parkingdetect()
{
    getRosParamForUpdate();
    throttle_ = 7;
    steer_control_value_ = -5;
    ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();
  	control_pub_.publish(control_msg);
    ros::Duration(3).sleep();
    throttle_ = 7;
    steer_control_value_ = 5;
     control_msg = makeControlMsg();
    control_pub_.publish(control_msg);
    ros::Duration(3).sleep();

}
