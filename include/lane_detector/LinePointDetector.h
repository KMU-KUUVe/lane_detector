#ifndef LINEPOINTDETECTOR_H
#define LINEPOINTDETECTOR_H

#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>

/**
 * @brief 양쪽 차선의 위치를 찾는 클래스
 * @detail 차선이 있는 이미지에서 하나의 평행한 선을 그엇을 때 그 선과 차선이 만나는 점을 찾는다. 그 점들을 가지고 스티어링 값을 계산할 수 있다.
 * 		   평행한 선을 어디에 그을 것인가는 y값으로 정할 수 있다.
 * 		   차선 인식은 이미지 중앙에서 바깥쪽으로 인식을 할수도 있고, 이미지 오른쪽/왼쪽 끝에서 안쪽으로 인식 할수도 있다.
 * 
 */
class LinePointDetector
{
public:
	LinePointDetector()
		: left_reset_flag_(true), right_reset_flag_(true)
	{}

	bool getLeftResetFlag() const;
	bool getRightResetFlag() const;
	int getContiDetectPixel() const;

	void setLeftResetFlag(const bool left_reset_flag);
	void setRightResetFlag(const bool right_reset_flag);
	void setContiDetectPixel(const int continuous_detect_pixel);

	/**
	 * 아래 함수들은 out에서 in으로 차선 인식을 하는 함수들이다
	 *
	 */

	/**
	 * 왼쪽 초기점 찾는 함수
	 *
	 * @param binary_img 차선을 찾을 roi 이진화 이미지
	 * @param detect_y_offset 차선을 찾을 roi 이미지 상의 y좌표 (0~100)
	 * @param last_point_x 초기 점을 다시 잡을때 사용하는 전에 인식된 점의 x좌표
	 *
	 * @return 찾은 왼쪽차선 초기점의 x좌표
	 */
	int find_L0_x_out2in(const cv::Mat& binary_img, const int detect_y_offset, const int last_point_x);

	/**
	 * 오른쪽 초기점 찾는 함수
	 *
	 */
	int find_R0_x_out2in(const cv::Mat& binary_img, const int detect_y_offset, const int last_point_x);

	/**
	 * 이전에 찾은 왼쪽 점을 사용해 다음점 찾는 함수
	 *
	 * @param binery_img 차선을 찾을 roi 이진화 이미지
	 * @param pre_point_x 전에 인식된 점의 x좌표
	 * @param detect_y_offset 차선을 찾을 roi 이미지 상의 y좌표 (0~100)
	 * @param line_pixel_threshold 하나의 차선을 나타내는 픽셀 수
	 *
	 * @return 찾은 왼쪽차선 다음점의 x좌표
	 */
	int find_LN_x_out2in(const cv::Mat& binary_img, const int pre_point_x, const int detect_y_offset, const int line_pixel_threshold);

	/**
	 * 이전에 찾은 오른쪽 점을 사용해 다음점 찾는 함수
	 *
	 */
	int find_RN_x_out2in(const cv::Mat& binary_img, const int pre_point_x, const int detect_y_offset, const int line_pixel_threshold);


	/**
	 * 아래 함수들은 in에서 out으로 차선 인식을 하는 함수들이다
	 *
	 */

	/**
	 * @param offset 안에서 인식이 시작되는 위치로, 센터를 기준으로의 offset을 의미한다.
	 * 		  		 이 함수는 left line을 인식하는 함수이므로 센터로부터 오른쪽 방향으로의 offset을 의미한다.
	 *
	 */
	int find_L0_x_in2out(const cv::Mat& binary_img, const int detect_y_offset, const int last_point_x, const int offset);

	/**
	 * @param offset 안에서 인식이 시작되는 위치로, 센터를 기준으로의 offset을 의미한다.
	 * 		  		 이 함수는 right line을 인식하는 함수이므로 센터로부터 왼쪽 방향으로의 offset을 의미한다.
	 *
	 */
	int find_R0_x_in2out(const cv::Mat& binary_img, const int detect_y_offset, const int last_point_x, const int offset);

	int find_LN_x_in2out(const cv::Mat& binary_img, const int pre_point_x, const int detect_y_offset, const int line_pixel_threshold, const int offset);

	int find_RN_x_in2out(const cv::Mat& binary_img, const int pre_point_x, const int detect_y_offset, const int line_pixel_threshold, const int offset);

private:
	bool laneIsDiscontinuous(const cv::Mat& binary_img, const int pre_point_x, const int detect_y_offset);

private:
	bool left_reset_flag_, right_reset_flag_;

	int continuous_detect_pixel_ = 30;	// 차선의 연속성을 판단할 때 기준이 되는 픽셀 갯수

};

#endif
