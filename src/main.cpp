#include "ros/ros.h"
#include <iostream>
#include "parking/ParkingNode.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking");

#if 0	// using camera
	ParkingNode parking_node;
	//do vehicle control first and then do parking.
	parking_node.parkingdetect();

	ros::spin();

#else	// using mp4 file
	ParkingNode parking_node("../challenge.mp4");
	parking_node.parkingdetect();
	parking_node.run_test();

#endif

	return 0;
}
