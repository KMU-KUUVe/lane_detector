#include "lane_detector/OutToInLaneDetector.h"

using namespace std;
using namespace cv;


OutToInLaneDetector::OutToInLaneDetector(const int width, const int height, const int steer_max_angle, const int detect_line_count)
  : LaneDetector(width, height, steer_max_angle, detect_line_count)
{}

void OutToInLaneDetector::cvtToRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size)
{
	Mat roi_gray_img;
	Mat roi_color_img = resized_img_(Rect(left_top.x, left_top.y, roi_size.width, roi_size.height));
	cvtColor(roi_color_img, roi_gray_img, COLOR_BGR2GRAY);
	threshold(roi_gray_img, roi_binary_img_, gray_bin_thres_, 255, THRESH_BINARY);
}

void OutToInLaneDetector::resetLeftPoint(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

	last_left_point_arr_[index].x = line_point_detector_arr_[index].find_L0_x_out2in(roi_binary_img_, detect_y_offset_arr_[index], last_left_point_arr_[index].x);
	last_left_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
}

void OutToInLaneDetector::resetRightPoint(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

	last_right_point_arr_[index].x = line_point_detector_arr_[index].find_R0_x_out2in(roi_binary_img_, detect_y_offset_arr_[index], last_right_point_arr_[index].x);
	last_right_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
}

void OutToInLaneDetector::updateNextPoint(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

	cur_right_point_arr_[index].x = line_point_detector_arr_[index].find_RN_x_out2in(roi_binary_img_, last_right_point_arr_[index].x, detect_y_offset_arr_[index], LINE_PIXEL_THRESHOLD);
	cur_right_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
	last_right_point_arr_[index].x = cur_right_point_arr_[index].x;

	cur_left_point_arr_[index].x = line_point_detector_arr_[index].find_LN_x_out2in(roi_binary_img_, last_left_point_arr_[index].x, detect_y_offset_arr_[index], LINE_PIXEL_THRESHOLD);
	cur_left_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
	last_left_point_arr_[index].x = cur_left_point_arr_[index].x;
}
