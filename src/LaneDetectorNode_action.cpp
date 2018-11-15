#include "lane_detector/LaneDetectorNode.h"

using namespace std;
using namespace cv;

LaneDetectorNode::LaneDetectorNode()
	:as_(nh_, "lane_detector", boost::bind(&LaneDetectorNode::actionCallback, this, _1), false)
{
	nh_ = ros::NodeHandle("~");
	as_.start();

	/* if NodeHangle("~"), then (write -> /lane_detector/write)	*/
	control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);

	getRosParamForUpdate();
}


void LaneDetectorNode::actionCallback(const state_cpp_msg::MissionPlannerGoalConstPtr& goal)
{
	cout << "lane detector actioniCallback called" << endl;
	mission_start = true;
	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &LaneDetectorNode::imageCallback, this);

	ros::Rate r(10);

	while(ros::ok()){
		if(mission_cleared){
			mission_start = false;
			state_cpp_msg::MissionPlannerResult result;
			steer_control_value_ = 0;
			throttle_ = 0;
			ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();
			control_pub_.publish(control_msg);
			destroyAllWindows();
			as_.setSucceeded(result);
			break;
		}
		r.sleep();
	}
}

void LaneDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
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

		getRosParamForUpdate();
		steer_control_value_ = laneDetecting();
		ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();
		control_pub_.publish(control_msg);
	}
}


void LaneDetectorNode::getRosParamForUpdate()
{
	nh_.getParam("throttle", throttle_);
	nh_.getParam("angle_factor", angle_factor_);
	nh_.getParam("stop_count", stop_count);
	nh_.getParam("steer_height", steer_height);
	
}


ackermann_msgs::AckermannDriveStamped LaneDetectorNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	control_msg.drive.steering_angle = steer_control_value_;
	control_msg.drive.speed = throttle_;
	return control_msg;
}


int LaneDetectorNode::laneDetecting()
{
	int ncols = frame.cols;
	int nrows = frame.rows;
	double angle_ = 0;
	
	int64 t1 = getTickCount();
	frame_count++;

	resize(frame, lane_frame, Size(ncols / resize_n, nrows / resize_n));
	img_mask = lanedetector.mask(lane_frame);
	lanedetector.filter_colors(img_mask, img_mask2);
	img_denoise = lanedetector.deNoise(img_mask2);

	double angle = lanedetector.steer_control(img_denoise, steer_height, 12, img_mask, zero_count);
	ROS_INFO("zero_count: %d", zero_count);
	if(zero_count > stop_count){
		mission_cleared= true;
		return 0;
	}
	else{
		mission_cleared = false;
	}

	int64 t2 = getTickCount();
	double ms = (t2 - t1) * 1000 / getTickFrequency();
	sum += ms;
	avg = sum / (double)frame_count;
	//cout << "it took :  " << ms << "ms." << "average_time : " << avg << " frame per second (fps) : " << 1000 / avg << endl;
	//ROS_INFO("it took : %6.2f [ms].  average_time : %6.2f [ms].  frame per second (fps) : %6.2f [frame/s].   steer angle : %5.2f [deg]\n", ms, avg, 1000 / avg , angle);
	
	//for add  factor to calculate left angle.
	if(angle <= 0){
		angle_ = (angle * angle_factor_) -5;
	}
	else{
		angle_ = angle * angle_factor_;
	}

	//for limit platform angle values.	

	if(angle_ > 26){
		angle_ = 26;
	}
	else if (angle_ < -26){
		angle_ = -26;
	}
	
	return angle_;
}



void LaneDetectorNode::parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);

	cv_img = cv_ptr->image;

	if (cv_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}
}


