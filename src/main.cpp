#include "ros/ros.h"
#include <iostream>
#include "parking/ParkingNode.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking");

#if 1	// using camera
	ParkingNode parking_node;
	//do vehicle control first and then do parking.
	std::cout << "start node" << std::endl;

	/*
	if(msg == 1){
		parking_node.parkingdetect_A();
	}else{
		parking_node.parkingdetect_B();
	}
	*/

	parking_node.parkingdetect_A();
	std::cout << "control node end" << std::endl;
	ros::spin();

#else	// using mp4 file
	ParkingNode parking_node("../challenge.mp4");
	parking_node.parkingdetect();
	parking_node.run_test();

#endif

	return 0;
}
