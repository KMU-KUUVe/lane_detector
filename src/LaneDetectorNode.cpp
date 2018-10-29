#include <cmath>
#include <string>
#include <vector>
#include <stdexcept>
#include "lane_detector/LaneDetectorNode.h"

using namespace std;
using namespace cv;

LaneDetectorNode::LaneDetectorNode()
{
	nh_ = ros::NodeHandle("~");
	/* if NodeHangle("~"), then (write -> /lane_detector/write)	*/
#if RC_CAR
	control_pub_ = nh_.advertise<std_msgs::String>("write", 10);
#elif	SCALE_PLATFORM
	control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);
#endif

#if DEBUG
	true_color_pub_ = nh_.advertise<sensor_msgs::Image>("truecolor", 10);
	final_bin_pub_ = nh_.advertise<sensor_msgs::Image>("final_bin", 10);
	bin_from_gray_pub_ = nh_.advertise<sensor_msgs::Image>("gray_bin", 10);
	bin_from_hsv_s_pub_ = nh_.advertise<sensor_msgs::Image>("hsv_s_bin", 10);
	printlog_pub_ = nh_.advertise<std_msgs::String>("printlog", 10);
#endif

#if WEBCAM
	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &LaneDetectorNode::imageCallback, this);
#elif	PROSILICA_GT_CAM
	image_sub_ = nh_.subscribe("/camera/image_raw", 1, &LaneDetectorNode::imageCallback, this);
#endif

	int resize_width = 0;
	int resize_height = 0;
	int steer_max_angle = 0;
	int detect_line_count = 0;

	getRosParamForConstValue(resize_width, resize_height, steer_max_angle, detect_line_count);

	lanedetector_ptr_ = unique_ptr<InToOutLaneDetector>(new InToOutLaneDetector(resize_width, resize_height, steer_max_angle, detect_line_count));

	getRosParamForUpdate();
}

void LaneDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
	Mat raw_img;
	try{
		parseRawimg(image, raw_img);
	} catch(const cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return ;
	} catch(const std::runtime_error& e) {
		cerr << e.what() << endl;
	}

    getRosParamForUpdate();

    int steer_control_value = lanedetector_ptr_->laneDetecting(raw_img);

#if	RC_CAR
	std_msgs::String control_msg = makeControlMsg(steer_control_value);
	printData(control_msg);
#elif	SCALE_PLATFORM
	ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();
	printData();
#endif

	control_pub_.publish(control_msg);

#if DEBUG
	true_color_pub_.publish(getDetectColorImg());
	final_bin_pub_.publish(getDetectFinalBinImg());
	bin_from_gray_pub_.publish(getDetectGrayBinImg());
	bin_from_hsv_s_pub_.publish(getDetectHsvSBinImg());
	printlog_pub_.publish(getPrintlog());
#endif
}

#if DEBUG
sensor_msgs::ImagePtr LaneDetectorNode::getDetectColorImg()
{
	Mat image;
	lanedetector_ptr_->getRoiColorImg(image);
	return cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
}

sensor_msgs::ImagePtr LaneDetectorNode::getDetectFinalBinImg()
{
	Mat image;
	lanedetector_ptr_->getRoiBinaryImg(image);
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

sensor_msgs::ImagePtr LaneDetectorNode::getDetectGrayBinImg()
{
	Mat image;
	lanedetector_ptr_->getRoiGrayBinImg(image);
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

sensor_msgs::ImagePtr LaneDetectorNode::getDetectHsvSBinImg()
{
	Mat image;
	lanedetector_ptr_->getRoiHsvSBinImg(image);
	return cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
}

std_msgs::String LaneDetectorNode::getPrintlog()
{
	string log;
 	log += "#### Algorithm Time #### \n";
	log += (string)"it took : " + to_string(lanedetector_ptr_->getOnceDetectTime()) + "ms, " + "avg: " + to_string(lanedetector_ptr_->getAvgDetectTime()) + " fps : " + to_string(1000 / lanedetector_ptr_->getAvgDetectTime()) + '\n';
	log += "#### Control #### \n";
	log += "steering angle: " + to_string(lanedetector_ptr_->getRealSteerAngle()) + '\n';
	log += "throttle: " + to_string(throttle_) + '\n';
	log += "#### Ros Param #### \n";
	log += "gray_bin_thres: " + to_string(lanedetector_ptr_->getGrayBinThres()) + '\n';
	log += "hsv_s_bin_thres: " + to_string(lanedetector_ptr_->getHsvSBinThres()) + '\n';
	log += "detect_line_count: " + to_string(lanedetector_ptr_->getDetectLineCount()) + '\n';
	for(int i = 0; i < lanedetector_ptr_->getDetectLineCount(); i++)
		log += "detect_y_offset_" + to_string(i+1) + ": " + to_string(lanedetector_ptr_->getDetectYOffset(i)) + '\n';
	log += "left_detect_offset: " + to_string(lanedetector_ptr_->getLeftDetectOffset()) + '\n';
	log += "right_detect_offset: " + to_string(lanedetector_ptr_->getRightDetectOffset()) + '\n';
	log += "steer_max_angle: " + to_string(lanedetector_ptr_->getSteerMaxAngle()) + '\n';
	log += "yaw_factor: " + to_string(lanedetector_ptr_->getYawFactor() * 100) + "% -> " + to_string(lanedetector_ptr_->getYawFactor()) + '\n';
	log += "lateral_factor: " + to_string(lanedetector_ptr_->getLateralFactor() * 100) + "% -> " + to_string(lanedetector_ptr_->getLateralFactor()) + '\n';
	log += "---------------------------------\n";

	std_msgs::String log_msg;
	log_msg.data = log;
	return log_msg;
}

#endif

void LaneDetectorNode::parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, Mat& cv_img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);

	cv_img = cv_ptr->image;

	if (cv_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}
}

void LaneDetectorNode::getRosParamForConstValue(int& width, int& height, int& steer_max_angle, int& detect_line_count)
{
	nh_.getParam("/image/resize/width", width);
	nh_.getParam("/image/resize/height", height);
	nh_.getParam("/hardware_control/platform/steer_max_angle", steer_max_angle);
	nh_.getParam("/detect/line_count", detect_line_count);
}

void LaneDetectorNode::getRosParamForUpdate()
{
	int paramArr[7];
	nh_.getParam("/image/binary_thres/gray", paramArr[0]);
	nh_.getParam("/image/binary_thres/hsv_s", paramArr[1]);
	nh_.getParam("/detect/offset/left", paramArr[2]);
	nh_.getParam("/detect/offset/right", paramArr[3]);
	nh_.getParam("/image/roi/top_location", paramArr[4]);
	nh_.getParam("/image/roi/bottom_location", paramArr[5]);
	nh_.getParam("/detect/continuous_pixel", paramArr[6]);

	double factor[2];
	nh_.getParam("/hardware_control/factor/yaw", factor[0]);
	nh_.getParam("/hardware_control/factor/lateral", factor[1]);
	nh_.getParam("/hardware_control/platform/throttle", throttle_);

	lanedetector_ptr_->setGrayBinThres(paramArr[0]);
	lanedetector_ptr_->setHsvSBinThres(paramArr[1]);
	lanedetector_ptr_->setLeftDetectOffset(paramArr[2]);
	lanedetector_ptr_->setRightDetectOffset(paramArr[3]);
	lanedetector_ptr_->setRoiTopLocation(paramArr[4]);
	lanedetector_ptr_->setRoiBottomLocation(paramArr[5]);
	lanedetector_ptr_->setContiDetectPixel(paramArr[6]);

	lanedetector_ptr_->setYawFactor(factor[0]);
	lanedetector_ptr_->setLateralFactor(factor[1]);

	vector<int> detect_y_offset;
	nh_.getParam("/detect/y_offset", detect_y_offset);

	int detect_line_count = lanedetector_ptr_->getDetectLineCount();

	if(detect_line_count != detect_y_offset.size())
	{
		// FIXME: throw exception? or assert? 
		return ;
	}

	for(int i = 0; i < detect_line_count; i++) {
		lanedetector_ptr_->setDetectYOffset(detect_y_offset[i], i);
	}
}

#if RC_CAR
std_msgs::String LaneDetectorNode::makeControlMsg(int steer)
{
	std_msgs::String control_msg;
	control_msg.data = string(to_string(steer)) + "," + string(to_string(throttle_)) + ","; // Make message
	return control_msg;
}

void LaneDetectorNode::printData(std_msgs::String control_msg)
{
 	cout << "#### Algorithm Time ####" << endl;
	cout << "it took : " << lanedetector_ptr_->getOnceDetectTime() << "ms, " << "avg: " << lanedetector_ptr_->getAvgDetectTime() << " fps : " << 1000 / lanedetector_ptr_->getAvgDetectTime() << endl;
	cout << "#### Control ####" << endl;
	cout << "steering angle: " << lanedetector_ptr_->getRealSteerAngle() << endl;
	cout << "control msg: " << control_msg.data << endl;
	cout << "#### Ros Param ####" << endl;
	cout << "gray_bin_thres: " << lanedetector_ptr_->getGrayBinThres() << endl;
	cout << "hsv_s_bin_thres: " << lanedetector_ptr_->getHsvSBinThres() << endl;
	cout << "detect_line_count: " << lanedetector_ptr_->getDetectLineCount() << endl;
	for(int i = 0; i < lanedetector_ptr_->getDetectLineCount(); i++)
		cout << "detect_y_offset_" << i+1 << ": " << lanedetector_ptr_->getDetectYOffset(i) << endl;
	cout << "left_detect_offset: " << lanedetector_ptr_->getLeftDetectOffset() << endl;
	cout << "right_detect_offset: " << lanedetector_ptr_->getRightDetectOffset() << endl;
	cout << "steer_max_angle: " << lanedetector_ptr_->getSteerMaxAngle() << endl;
	cout << "yaw_factor: " << lanedetector_ptr_->getYawFactor() * 100 << "% -> " << lanedetector_ptr_->getYawFactor() << endl;
	cout << "lateral_factor: " << lanedetector_ptr_->getLateralFactor() * 100 << "% -> " << lanedetector_ptr_->getLateralFactor() << endl;
	cout << "---------------------------------" << endl;
}

#elif SCALE_PLATFORM
ackermann_msgs::AckermannDriveStamped LaneDetectorNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	//control_msg.drive.steering_angle = steer_control_value;
	control_msg.drive.steering_angle = lanedetector_ptr_->getRealSteerAngle();
	control_msg.drive.speed = throttle_;
	return control_msg;
}

void LaneDetectorNode::printData()
{
 	cout << "#### Algorithm Time ####" << endl;
	cout << "it took : " << lanedetector_ptr_->getOnceDetectTime() << "ms, " << "avg: " << lanedetector_ptr_->getAvgDetectTime() << " fps : " << 1000 / lanedetector_ptr_->getAvgDetectTime() << endl;
	cout << "#### Control ####" << endl;
	cout << "steering angle: " << lanedetector_ptr_->getRealSteerAngle() << endl;
	cout << "throttle: " << throttle_ << endl;
	cout << "#### Ros Param ####" << endl;
	cout << "gray_bin_thres: " << lanedetector_ptr_->getGrayBinThres() << endl;
	cout << "hsv_s_bin_thres: " << lanedetector_ptr_->getHsvSBinThres() << endl;
	cout << "detect_line_count: " << lanedetector_ptr_->getDetectLineCount() << endl;
	for(int i = 0; i < lanedetector_ptr_->getDetectLineCount(); i++)
		cout << "detect_y_offset_" << i+1 << ": " << lanedetector_ptr_->getDetectYOffset(i) << endl;
	cout << "left_detect_offset: " << lanedetector_ptr_->getLeftDetectOffset() << endl;
	cout << "right_detect_offset: " << lanedetector_ptr_->getRightDetectOffset() << endl;
	cout << "steer_max_angle: " << lanedetector_ptr_->getSteerMaxAngle() << endl;
	cout << "yaw_factor: " << lanedetector_ptr_->getYawFactor() * 100 << "% -> " << lanedetector_ptr_->getYawFactor() << endl;
	cout << "lateral_factor: " << lanedetector_ptr_->getLateralFactor() * 100 << "% -> " << lanedetector_ptr_->getLateralFactor() << endl;
	cout << "---------------------------------" << endl;
}
#endif
