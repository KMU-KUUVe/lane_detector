#include "lane_detector/LaneDetectorNode.h"

using namespace std;
using namespace cv;

LaneDetectorNode::LaneDetectorNode()
{
	nh_ = ros::NodeHandle("~");

	/* if NodeHangle("~"), then (write -> /lane_detector/write)	*/
	control_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann", 10);

	image_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &LaneDetectorNode::imageCallback, this);

	getRosParamForUpdate();
}


LaneDetectorNode::LaneDetectorNode(String path)
	: test_video_path(path)
{}


void LaneDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
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

	steer_control_value_ = laneDetecting();

	ackermann_msgs::AckermannDriveStamped control_msg = makeControlMsg();

	control_pub_.publish(control_msg);
}


void LaneDetectorNode::getRosParamForUpdate()
{
	nh_.getParam("throttle", throttle_);
	nh_.getParam("angle_factor", angle_factor_);
}


ackermann_msgs::AckermannDriveStamped LaneDetectorNode::makeControlMsg()
{
	ackermann_msgs::AckermannDriveStamped control_msg;
	//control_msg.drive.steering_angle = steer_control_value;
	control_msg.drive.steering_angle = steer_control_value_;
	control_msg.drive.speed = throttle_;
	return control_msg;
}


int LaneDetectorNode::laneDetecting()
{
	int ncols = frame.cols;
	int nrows = frame.rows;


	int64 t1 = getTickCount();
	frame_count++;

	// ȭ�� ũ�� ���� -> �ػ� �����Ͽ� ���ӵ� ���
	resize(frame, lane_frame, Size(ncols / resize_n, nrows / resize_n));

	img_denoise = lanedetector.deNoise(lane_frame);


	lanedetector.filter_colors(img_denoise, img_mask2);

	// Ȯ�� ����
	//Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
	//dilate(img_mask2, img_mask2, mask, Point(-1, -1), 3);

	// �󺧸�
	// lanedetector.DrawLabelingImage(img_mask2);

	imshow("img_mask2", img_mask2);
	img_edges = lanedetector.edgeDetector(img_mask2);



	// Mask the image so that we only get the ROI
	img_mask = lanedetector.mask(img_edges,Mask_method);



	imshow("img_mask", img_mask);

	// Obtain Hough lines in the cropped image
	lines = lanedetector.houghLines(img_mask);


	// Separate lines into left and right lines
	left_right_lines = lanedetector.lineSeparation(lines, img_mask);

	/*
	// Ȯ��
	for (j = 0; j < left_right_lines[0].size(); j++)
	{
	circle(frame, Point(left_right_lines[0][j][0], left_right_lines[0][j][1]), 5, Scalar(255, 0, 0), 5);
	circle(frame, Point(left_right_lines[0][j][2], left_right_lines[0][j][3]), 5, Scalar(0, 0, 255), 5);


	}

	for (j = 0; j < left_right_lines[1].size(); j++)
	{

	circle(frame, Point(left_right_lines[1][j][0], left_right_lines[1][j][1]), 5, Scalar(0, 255, 0), 5);
	circle(frame, Point(left_right_lines[1][j][2], left_right_lines[1][j][3]), 5, Scalar(0, 255, 0), 5);

	}
	 */


	line(lane_frame, Point(10, 0), Point(10, 20), Scalar(0, 0, 255), 5);

	// Apply regression to obtain only one line for each side of the lane
	lane = lanedetector.regression(left_right_lines, lane_frame, angle);  // frame -> img_mask

	// Predict the turn by determining the vanishing point of the the lines
	turn = lanedetector.predictTurn();

	// Plot lane detection
	flag_plot = lanedetector.plotLane(lane_frame, lane, turn);



	int64 t2 = getTickCount();
	double ms = (t2 - t1) * 1000 / getTickFrequency();
	sum += ms;
	avg = sum / (double)frame_count;
	//cout << "it took :  " << ms << "ms." << "average_time : " << avg << " frame per second (fps) : " << 1000 / avg << endl;
	waitKey(3);
	ROS_INFO("it took : %6.2f [ms].  average_time : %6.2f [ms].  frame per second (fps) : %6.2f [frame/s].   steer angle : %5.2f [deg]\n", ms, avg, 1000 / avg , angle);

	return angle * angle_factor_;
}



void LaneDetectorNode::parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);

	cv_img = cv_ptr->image;

	if (cv_img.empty()) {
		throw std::runtime_error("frame is empty!");
	}
}


bool LaneDetectorNode::run_test()
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

		// ȭ�� ũ�� ���� -> �ػ� �����Ͽ� ���ӵ� ���
		resize(frame, frame, Size(ncols / resize_n, nrows / resize_n));

		img_denoise = lanedetector.deNoise(frame);


		lanedetector.filter_colors(img_denoise, img_mask2);

		// Ȯ�� ����
		//Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
		//dilate(img_mask2, img_mask2, mask, Point(-1, -1), 3);

		// �󺧸�
		// lanedetector.DrawLabelingImage(img_mask2);

		imshow("img_mask2", img_mask2);
		img_edges = lanedetector.edgeDetector(img_mask2);



		// Mask the image so that we only get the ROI
		img_mask = lanedetector.mask(img_edges,Mask_method);



		imshow("img_mask", img_mask);

		// Obtain Hough lines in the cropped image
		lines = lanedetector.houghLines(img_mask);


		// Separate lines into left and right lines
		left_right_lines = lanedetector.lineSeparation(lines, img_mask);

		/*
		// Ȯ��
		for (j = 0; j < left_right_lines[0].size(); j++)
		{
		circle(frame, Point(left_right_lines[0][j][0], left_right_lines[0][j][1]), 5, Scalar(255, 0, 0), 5);
		circle(frame, Point(left_right_lines[0][j][2], left_right_lines[0][j][3]), 5, Scalar(0, 0, 255), 5);


		}

		for (j = 0; j < left_right_lines[1].size(); j++)
		{

		circle(frame, Point(left_right_lines[1][j][0], left_right_lines[1][j][1]), 5, Scalar(0, 255, 0), 5);
		circle(frame, Point(left_right_lines[1][j][2], left_right_lines[1][j][3]), 5, Scalar(0, 255, 0), 5);

		}
		 */


		line(frame, Point(10, 0), Point(10, 20), Scalar(0, 0, 255), 5);

		// Apply regression to obtain only one line for each side of the lane
		lane = lanedetector.regression(left_right_lines, frame, angle);  // frame -> img_mask

		// Predict the turn by determining the vanishing point of the the lines
		turn = lanedetector.predictTurn();

		// Plot lane detection
		flag_plot = lanedetector.plotLane(frame, lane, turn);



		int64 t2 = getTickCount();
		double ms = (t2 - t1) * 1000 / getTickFrequency();
		sum += ms;
		avg = sum / (double)frame_count;
		waitKey(25);
		//cout << "it took :  " << ms << "ms." << "average_time : " << avg << " frame per second (fps) : " << 1000 / avg << endl;

		printf("it took : %6.2f [ms].  average_time : %6.2f [ms].  frame per second (fps) : %6.2f [frame/s].   steer angle : %5.2f [deg]\n", ms, avg, 1000 / avg , angle);
	}

}
