#include <ros/ros.h>
#include "lane_detector/LaneDetectorNode.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lane_detector");

#if 1
	//LaneDetector lane_detector(960/2, 540/2, 45);
	LaneDetectorNode lane_detector_node;

#else
	VideoCapture cap(1);
	//cap.open("cameraimage_color_camera3.mp4");

	if (!cap.isOpened())
	{
		cout << "Not opened cap" << endl;
		return -1;
	}
#endif

	ros::spin();
	return 0;
}
