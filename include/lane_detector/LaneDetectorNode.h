#ifndef LANEDETECTORNODE_H
#define LANEDETECTORNODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <signal.h>
#include <memory>
#include "lane_detector/InToOutLaneDetector.h"
#include "lane_detector/ConditionalCompile.h"

/**
 * @brief LaneDetector를 ROS 노드화한 클래스이다
 * @details ROS Node를 나타내는 Class로, Subscriber/Publisher/Rosbag Logger 등을 담고 있다.
 * 
 */
class LaneDetectorNode
{
public:
	LaneDetectorNode();

	/**
	 * @brief 카메라로부터 들어온 이미지를 Subscribe 했을 때 호출되는 Callback 함수
	 * 
	 * @param image 카메라 드라이버 노드에서 보낸 이미지를 받아 포인팅하고있는 포인터
	 * 
	 */
	void imageCallback(const sensor_msgs::ImageConstPtr& image);

private:
	/**
	 * @brief 차선 인식과 관련된 파라미터 중 동적으로 바뀌지 않는 값들을 읽어오는 함수
	 * 
	 * @details 파라미터는 config/*.yaml 파일에서 나타내고있고, *.launch파일에서 load하게 된다. 이 함수를 통해 읽어들이는 값은 맨 처음 이 노드를 초기화할때만 호출되므로, rosparam을 이용해서 노드 실행중에 동적으로 파라미터값을 수정할 수 없다.
	 * 
	 */
	void getRosParamForConstValue(int& width, int& height, int& steer_max_angle, int& detect_line_count);

	/**
	 * @brief 차선 인식과 관련된 파라미터 중 동적으로 바뀔 수 있는 값들을 읽어오는 함수
	 * 
	 * @details 이 함수는 주기적으로 계속 호출되므로, rosparam을 통해 노드 실행중 동적으로 값들을 바꾸면서 테스트가 가능하다.
	 */
	void getRosParamForUpdate();

	/**
	 * @brief Ros 통신에서 사용하는 이미지 타입을 Opencv의 Mat 타입으로 변환해주는 함수
	 * 
	 */
	void parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img);
#if DEBUG
	sensor_msgs::ImagePtr getDetectColorImg();
	sensor_msgs::ImagePtr getDetectFinalBinImg();
	sensor_msgs::ImagePtr getDetectGrayBinImg();
	sensor_msgs::ImagePtr getDetectHsvSBinImg();

	std_msgs::String getPrintlog();
#endif

#if RC_CAR
	std_msgs::String makeControlMsg(int steer);
	void printData(std_msgs::String control_msg);
#elif SCALE_PLATFORM
	/**
	 * @brief 계산된 컨트롤값(steering, throttle)을 가지고 Controller노드와 약속한 타입대로 메시지를 만들어주는 함수
	 * 
	 * @details 이 메시지는 Controller Node를 위해 Publish된다.
	 * 
	 */
	ackermann_msgs::AckermannDriveStamped makeControlMsg();
	void printData();
#endif

private:
	ros::NodeHandle nh_;
	ros::Publisher control_pub_;	// Controll 메시지를 Publish하는 Publisher
#if DEBUG
	ros::Publisher true_color_pub_;
	ros::Publisher final_bin_pub_;
	ros::Publisher bin_from_gray_pub_;
	ros::Publisher bin_from_hsv_s_pub_;

	ros::Publisher printlog_pub_;
#endif
	ros::Subscriber image_sub_;		// 가공되지 않은 raw image 메시지를 Subscribe하는 Subscriber

	int throttle_ = 0;

	std::unique_ptr<InToOutLaneDetector> lanedetector_ptr_;		// 차선 인식 알고리즘을 담고있는 클래스 객체를 가리키는 포인터. ROS와 별개로 만들어진 LaneDetector와의 인터페이스 역할을 한다.
};

#endif
