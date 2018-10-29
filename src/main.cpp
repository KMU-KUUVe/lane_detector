#include "ros/ros.h"
#include <iostream>
#include "lane_detector/LaneDetectorNode.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lane_detector");

#if 1	// using camera
	LaneDetectorNode lane_detector_node;

	ros::spin();

#else	// using mp4 file
	LaneDetectorNode lane_detector_node("../test/challenge.mp4");

	lane_detector_node.run_test();

#endif

	return 0;
}
