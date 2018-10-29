#include "lane_detector/LaneDetector.h"

using namespace std;
using namespace cv;

LaneDetector::LaneDetector(const int width, const int height, const int steer_max_angle, const int detect_line_count)
  : RESIZE_WIDTH_(width), RESIZE_HEIGHT_(height), STEER_MAX_ANGLE_(steer_max_angle), DETECT_LINE_COUNT_(detect_line_count)
{
  detect_y_offset_arr_ = unique_ptr<int[]>(new int[DETECT_LINE_COUNT_]);
  last_right_point_arr_ = unique_ptr<Point[]>(new Point[DETECT_LINE_COUNT_]);
  last_left_point_arr_ = unique_ptr<Point[]>(new Point[DETECT_LINE_COUNT_]);
  cur_right_point_arr_ = unique_ptr<Point[]>(new Point[DETECT_LINE_COUNT_]);
  cur_left_point_arr_ = unique_ptr<Point[]>(new Point[DETECT_LINE_COUNT_]);
  lane_middle_arr_ = unique_ptr<Point[]>(new Point[DETECT_LINE_COUNT_]);
  line_point_detector_arr_ = unique_ptr<LinePointDetector[]>(new LinePointDetector[DETECT_LINE_COUNT_]);

  for(int index = 0; index < DETECT_LINE_COUNT_; index++) {
    last_left_point_arr_[index] = Point(0, 0);
    last_right_point_arr_[index] = Point(0, 0);
  }
}

void LaneDetector::setGrayBinThres(const int bin_thres) { gray_bin_thres_ = bin_thres; }
bool LaneDetector::setDetectYOffset(const int detect_y_offset, const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  detect_y_offset_arr_[index] = detect_y_offset;
  return true;
}
void LaneDetector::setYawFactor(const double yaw_factor) { yaw_factor_ = yaw_factor; }
void LaneDetector::setLateralFactor(const double lateral_factor) { lateral_factor_ = lateral_factor; }
void LaneDetector::setRoiTopLocation(const int top_rate) { roi_top_location_ = top_rate; }
void LaneDetector::setRoiBottomLocation(const int bottom_rate) { roi_bottom_location_ = bottom_rate; }
void LaneDetector::setContiDetectPixel(const int continuous_detect_pixel)
{
  for(int i = 0; i < DETECT_LINE_COUNT_; i++)
    line_point_detector_arr_[i].setContiDetectPixel(continuous_detect_pixel);
}



int LaneDetector::getWidth() const { return RESIZE_WIDTH_; }
int LaneDetector::getHeight() const { return RESIZE_HEIGHT_; }
int LaneDetector::getGrayBinThres() const { return gray_bin_thres_; }
int LaneDetector::getDetectLineCount() const { return DETECT_LINE_COUNT_; }
int LaneDetector::getDetectYOffset(const int index) const
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  return detect_y_offset_arr_[index];
}
int LaneDetector::getSteerMaxAngle() const { return STEER_MAX_ANGLE_; }
int LaneDetector::getRoiTopLocation() const { return roi_top_location_; }
int LaneDetector::getRoiBottomLocation() const { return roi_bottom_location_; }
int LaneDetector::getContiDetectPixel() const { return line_point_detector_arr_[0].getContiDetectPixel(); }
double LaneDetector::getYawFactor() const { return yaw_factor_; }
double LaneDetector::getLateralFactor() const { return lateral_factor_; }
double LaneDetector::getOnceDetectTime() const { return once_detect_time_; }
double LaneDetector::getAvgDetectTime() const { return detect_avg_time_; }
void LaneDetector::getRoiColorImg(Mat& img) { img = resized_img_; }
void LaneDetector::getRoiBinaryImg(Mat& img) { img = roi_binary_img_; }


double LaneDetector::calculateYawError(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

	const int dx = lane_middle_arr_[index].x - roi_binary_img_.cols / 2;
	const int dy = roi_binary_img_.rows - lane_middle_arr_[index].y;
	return atan2(dx, dy) * 180 / CV_PI;
}

double LaneDetector::calculateLateralError(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

	return lane_middle_arr_[index].x - roi_binary_img_.cols / 2;
}

void LaneDetector::calculateOnceDetectTime(const int64 start_time, const int64 finish_time)
{
	once_detect_time_ = (finish_time - start_time) * 1000 / getTickFrequency();
}

void LaneDetector::calculateAvgDetectTime()
{
	sum_of_detect_time_ += once_detect_time_;
	detect_avg_time_ = sum_of_detect_time_ / frame_count_;
}

bool LaneDetector::detectedOnlyOneLine(const int index) const
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

	return abs(cur_right_point_arr_[index].x - cur_left_point_arr_[index].x) < 15;
}

void LaneDetector::visualizeLine(const int index) const
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

	line(resized_img_, cur_right_point_arr_[index] + Point(0, RESIZE_HEIGHT_ * (double)roi_top_location_ / 100), cur_left_point_arr_[index] + Point(0, RESIZE_HEIGHT_ * (double)roi_top_location_ / 100), Scalar(0, 255, 0), 5);
	line(resized_img_, lane_middle_arr_[index] + Point(0, RESIZE_HEIGHT_ * (double)roi_top_location_ / 100), Point(RESIZE_WIDTH_ / 2, RESIZE_HEIGHT_ * (double)roi_bottom_location_ / 100), Scalar(0, 0, 255), 5);
}

void LaneDetector::showImg() const
{
	imshow("binary img", roi_binary_img_);
	imshow("frame", resized_img_);
	waitKey(3);
}

int LaneDetector::calculateSteerValue(const int center_steer_control_value, const int max_steer_control_value)
{
	int steer_control_value = 0;	// for parsing double value to int
	const int steer_offset = max_steer_control_value - center_steer_control_value;  // center ~ max

//TODO: Modify steering altorithm
/*
	steer_angle_ = yaw_factor_ * yaw_error_ + lateral_factor_ * lateral_error_;

	if(steer_angle_ < STEER_MAX_ANGLE_ && steer_angle_ > (-1) * STEER_MAX_ANGLE_) {
		steer_control_value = static_cast<int>(center_steer_control_value + (steer_offset / STEER_MAX_ANGLE_) * steer_angle_);
	}
	else if(steer_angle_ >= STEER_MAX_ANGLE_) {
		steer_control_value = center_steer_control_value + steer_offset;
	}
	else if(steer_angle_ <= (-1) * STEER_MAX_ANGLE_) {
		steer_control_value = center_steer_control_value - steer_offset;
	}
  */

	int tmp_control_value = yaw_error_ * yaw_factor_ + lateral_error_ * lateral_factor_;

  if(yaw_error_ * yaw_factor_ < STEER_MAX_ANGLE_ && yaw_error_ * yaw_factor_ > (-1) * STEER_MAX_ANGLE_)
  		steer_control_value = static_cast<int>(center_steer_control_value + steer_offset / STEER_MAX_ANGLE_ * (yaw_error_ * yaw_factor_) + lateral_error_ * lateral_factor_);
  	else if(yaw_error_ * yaw_factor_ >= STEER_MAX_ANGLE_) {
  		steer_control_value = center_steer_control_value + steer_offset;
  		yaw_error_ = STEER_MAX_ANGLE_ / yaw_factor_;		// for print angle on console
  	}
  	else if(yaw_error_ * yaw_factor_ <= (-1) * STEER_MAX_ANGLE_) {
  		steer_control_value = center_steer_control_value - steer_offset;
  		yaw_error_ = (-1) * STEER_MAX_ANGLE_ / yaw_factor_;
  }

	return steer_control_value;
}

// int LaneDetector::getRealSteerAngle() const { return steer_angle_; }
int LaneDetector::getRealSteerAngle() const { return yaw_error_ * yaw_factor_; }

// for testFunction
bool LaneDetector::haveToResetLeftPoint(const int index) const
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  return line_point_detector_arr_[index].getLeftResetFlag();
}
bool LaneDetector::haveToResetRightPoint(const int index) const
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  return line_point_detector_arr_[index].getRightResetFlag();
}

Point LaneDetector::detectLaneCenter(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  return Point((cur_right_point_arr_[index].x + cur_left_point_arr_[index].x) / 2, roi_binary_img_.rows * detect_y_offset_arr_[index] / 100);
}

void LaneDetector::reservePointReset(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

	line_point_detector_arr_[index].setLeftResetFlag(true);
	line_point_detector_arr_[index].setRightResetFlag(true);
}

void LaneDetector::preprocessImg(const cv::Mat& raw_img)
{
	resize(raw_img, resized_img_, Size(RESIZE_WIDTH_, RESIZE_HEIGHT_));

	cvtToRoiBinaryImg(Point(0, RESIZE_HEIGHT_ * (double)roi_top_location_ / 100), Size(RESIZE_WIDTH_, RESIZE_HEIGHT_ * (1 - (double)(roi_top_location_ + (100 - roi_bottom_location_)) / 100)));
}

void LaneDetector::findLanePoints()
  throw(my_out_of_range)
{
  for(int index = 0; index < DETECT_LINE_COUNT_; index++) {

  	if(haveToResetLeftPoint(index)) {
  		resetLeftPoint(index);
  	}

  	if (haveToResetRightPoint(index)) {
  		resetRightPoint(index);
  	}

  	updateNextPoint(index);
  }

}

/**
 * 각 line들의 에러값을 계산 후 평균값을 사용
 *
 */
void LaneDetector::findSteering()
  throw(my_out_of_range)
{
  for(int index = 0; index < DETECT_LINE_COUNT_; index++) {
  	lane_middle_arr_[index] = detectLaneCenter(index);
  }

  double yaw_error_sum = 0.0;
  double lateral_error_sum = 0.0;

  for(int index = 0; index < DETECT_LINE_COUNT_; index++) {
    yaw_error_sum += calculateYawError(index);
    lateral_error_sum += calculateLateralError(index);
  }

	yaw_error_ = yaw_error_sum / DETECT_LINE_COUNT_;
	lateral_error_ = lateral_error_sum / DETECT_LINE_COUNT_;

}

void LaneDetector::calDetectingTime(const int64 start_time, const int64 finish_time)
{
	calculateOnceDetectTime(start_time, finish_time);

	calculateAvgDetectTime();
}

void LaneDetector::checkPointReset()
  throw(my_out_of_range)
{
  for(int index = 0; index < DETECT_LINE_COUNT_; index++)
  	if (detectedOnlyOneLine(index))
  		reservePointReset(index);
}

void LaneDetector::visualizeAll()
  throw(my_out_of_range)
{
  for(int index = 0; index < DETECT_LINE_COUNT_; index++)
  	visualizeLine(index);

	showImg();
}

int LaneDetector::laneDetecting(const cv::Mat& raw_img)
{
	const int64 start_time = getTickCount();
	frame_count_++;

	preprocessImg(raw_img);

	findLanePoints();

	findSteering();

	const int64 finish_time = getTickCount();

	calDetectingTime(start_time, finish_time);

  checkPointReset();

	visualizeAll();

#if RC_CAR
// arduino steering range: 1100 < steer < 1900
	return calculateSteerValue(1500, 1900);
#elif SCALE_PLATFORM
// platform steering range: -26 < steer < +26
	return calculateSteerValue(0, 26);
#endif
}
