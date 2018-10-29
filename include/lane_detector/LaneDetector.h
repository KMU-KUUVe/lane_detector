#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include "opencv2/opencv.hpp"
#include "ConditionalCompile.h"
#include "LinePointDetector.h"
#include "MyException.h"
#include <memory>

/**
 * @brief 차선 인식을 위한 클래스
 * 
 */
class LaneDetector
{

public:
//TODO: caller have to pass detect_line_count value
	LaneDetector(const int width, const int height, const int steer_max_angle, const int detect_line_count);

	void setGrayBinThres(const int bin_thres);
	bool setDetectYOffset(const int detect_y_offset, const int index) throw(my_out_of_range);
	void setYawFactor(const double yaw_factor);
	void setLateralFactor(const double lateral_factor);
	void setRoiTopLocation(const int top_rate);
	void setRoiBottomLocation(const int bottom_rate);
	void setContiDetectPixel(const int continuous_detect_pixel);

	int getWidth() const;
	int getHeight() const;
	int getGrayBinThres() const;
	int getDetectLineCount() const;
	int getDetectYOffset(const int index) const throw(my_out_of_range);
	int getSteerMaxAngle() const;
	int getRealSteerAngle() const;
	int getRoiTopLocation() const;
	int getRoiBottomLocation() const;
	int getContiDetectPixel() const;
	double getYawFactor() const;
	double getLateralFactor() const;
	double getOnceDetectTime() const;
	double getAvgDetectTime() const;
	void getRoiColorImg(cv::Mat& img);
	void getRoiBinaryImg(cv::Mat& img);

	/**
	 * @brief 가공되지 않은 raw image를 roi처리하고 binary image로 만드는 함수
	 * 
	 * @details 순수 가상함수로 선언되어 있어 LaneDetector 클래스를 상속받는 클래스에서 정의한다.
	 * 			차선 중 흰색선은 gray image에서 인식하고, 노란색선은 hsv image를 통해 인식한다.
	 * 
	 */
	virtual void cvtToRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size) = 0;

	/**
	 * @brief 이 함수는 외부에서 호출되는 함수로, raw image를 받아 최종적으로 steering값을 반환해주는 함수
	 * 
	 * @return steering value
	 * 
	 */
	int laneDetecting(const cv::Mat& raw_img);

protected:
	/**
	 * 아래 선언된 함수들 대부분이 index라는 인자를 가지고 있다. 이것은 차선 인식하는 선이 여러개일 수 있기 때문이다.(파라미터로 설정)
	 * 
	 */


	/**
	 * @brief 차선 왼쪽(Left) or 오른쪽(Right) 포인트를 초기화하는 함수
	 * 
	 * @details '초기화'한다는 의미를 조금 자세히 설명하면,
	 * 			 차선 인식을 할 때 두가지 방법이 있다.
	 * 			 첫번째는 맨 처음 차선을 인식하거나 전에 인식한 차선 위치의 신뢰성이 없을 때 차선을 찾는 방법과
	 * 			 두번째는 직전에 인식한 차선 위치가 신뢰성이 높아 그것을 통해 현 차선의 위치를 찾는 방법.
	 * 			 '초기화'한다는 것은 첫번째 방법으로 차선을 인식한다는 의미다.
	 * 
	 */
	virtual void resetLeftPoint(const int index) throw(my_out_of_range) = 0;
	virtual void resetRightPoint(const int index) throw(my_out_of_range) = 0;

	/**
	 * @brief 이전 점을 기반으로 차선의 왼쪽(Left) or 오른쪽(Right) 포인트를 찾는 함수
	 * 
	 * @details 위에서 소개한 방법 중 두번째 방법으로 차선을 인식한다.
	 * 
	 */
	virtual void updateNextPoint(const int index) throw(my_out_of_range) = 0;

	/**
	 * @brief 인식한 차선의 위치를 기반으로 차량의 yaw error를 계산하는 함수
	 * 
	 */
	double calculateYawError(const int index) throw(my_out_of_range);

	/**
	 * @brief 인식한 차선의 위치를 기반으로 차량의 lateral error를 계산하는 함수
	 * 
	 */
	double calculateLateralError(const int index) throw(my_out_of_range);

	/**
	 * @brief 차선 인식 알고리즘의 순간 fps를 계산하는 함수
	 * 
	 */
	void calculateOnceDetectTime(const int64 start_time, const int64 finish_time);

	/**
	 * @brief 차선 인식 알고리즘의 평균 fps를 계산하는 함수
	 * 
	 */
	void calculateAvgDetectTime();

	/**
	 * @brief 두개의 차선을 제대로 인식했는가를 판단해주는 함수
	 * 
	 * @return 두개의 차선을 제대로 인식했다면 false, 인식하지 못했다면 true
	 * 
	 */
	bool detectedOnlyOneLine(const int index) const throw(my_out_of_range);

	/**
	 * @brief raw image위에 우리가 인식한 차선을 시각화하는 함수
	 * 
	 */
	void visualizeLine(const int index) const throw(my_out_of_range);

	/**
	 * @brief 디버깅할때 필요한 이미지들을 띄우는 함수
	 * 
	 * @details Opencv를 기반으로 하고 있으며, 띄우는 이미지는 총 4개로 color raw image, binary(이하 bin) image from gray, bin image from hsv, result bin image 가 있다.
	 * 
	 */
	virtual void showImg() const;

	/**
	 * @brief 제어할 때 사용할 steering값을 계산하는 함수
	 * 
	 * @details yaw_error와 lateral_error를 사용하여 최종 제어 steering을 계산하는 알고리즘이 들어있다.
	 * 
	 */
	int calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value);

	/**
	 * @brief 차선의 왼쪽(Left) or 오른쪽(Right) 포인트를 초기화 해야 하는지 체크하는 함수
	 * 
	 * @details 초기화의 의미를 알고자 한다면 'resetLeftPoint'함수 선언의 설명을 보길 바란다.
	 * 
	 * @return 초기화를 해야한다면 true, 아니라면 false
	 * 
	 */
	bool haveToResetLeftPoint(const int index) const throw(my_out_of_range);
	bool haveToResetRightPoint(const int index) const throw(my_out_of_range);

	/**
	 * @brief 차선의 중앙을 인식하는 함수
	 * 
	 */
	virtual cv::Point detectLaneCenter(const int index) throw(my_out_of_range);

	/**
	 * @brief 차선 인식 포인트 초기화를 예약하는 함수
	 * 
	 * @details 이 함수가 필요한 이유는 초기화를 해야하는지 판단하는 시점과 실제 초기화를 하는 시점이 다르기 때문이다.
	 * 
	 */
	void reservePointReset(const int index) throw(my_out_of_range);

	// wrapper function for lane detection
	// these functions are called on `laneDetecting` function
	/**
	 * @brief 유익하게 public으로 선언된 laneDetecting함수에서 순차적으로 호출하는 함수들
	 * 
	 * @details laneDetecting 함수에서 이 함수들을 호출하고, 이 함수들 내부적으로 위에 선언된 함수들을 호출한다고 보면 된다.
	 * 
	 */
	void preprocessImg(const cv::Mat& raw_img);
	void findLanePoints() throw(my_out_of_range);
	void findSteering() throw(my_out_of_range);
	void calDetectingTime(const int64 start_time, const int64 finish_time);
	void visualizeAll() throw(my_out_of_range);
	void checkPointReset() throw(my_out_of_range);

protected:
	// 화면 resize값
	const int RESIZE_WIDTH_ = 480;
	const int RESIZE_HEIGHT_ = 270;

	// Roi top and bottom(y) location
	// 0 is top of raw_img and 100 is bottom of raw_img
	int roi_top_location_ = 50;
	int roi_bottom_location_ = 100;

	// 한 직선 보는 임계값
	const int LINE_PIXEL_THRESHOLD = 11;

	// 차선 검출 시 사용하는 수평 라인의 갯수
	const int DETECT_LINE_COUNT_ = 1;

	// 라인 y 좌표 비율 컨테이너(0~100)
	std::unique_ptr<int[]> detect_y_offset_arr_;

	// LaneDetector
	int gray_bin_thres_ = 210;
	double yaw_factor_ = 0.5;
	double lateral_factor_ = 0.5;
#if RC_CAR
	const int STEER_MAX_ANGLE_ = 45;
#elif SCALE_PLATFORM
	const int STEER_MAX_ANGLE_ = 27;
#endif

	// LaneDetector below all
	cv::Mat resized_img_;		// resized image by (width, height)
	cv::Mat roi_binary_img_;

	std::unique_ptr<cv::Point[]> last_right_point_arr_;
	std::unique_ptr<cv::Point[]> last_left_point_arr_;

	std::unique_ptr<cv::Point[]> cur_right_point_arr_;
	std::unique_ptr<cv::Point[]> cur_left_point_arr_;

	std::unique_ptr<cv::Point[]> lane_middle_arr_;

	double yaw_error_;
	double lateral_error_;

	double steer_angle_;	// calculated real steer angle

	int frame_count_ = 0;	// for getting average fps
	double sum_of_detect_time_ = 0;
	double once_detect_time_ = 0;
	double detect_avg_time_ = 0;

	std::unique_ptr<LinePointDetector[]> line_point_detector_arr_;
};

#endif
