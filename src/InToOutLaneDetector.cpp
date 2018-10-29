#include "lane_detector/InToOutLaneDetector.h"

using namespace std;
using namespace cv;


InToOutLaneDetector::InToOutLaneDetector(const int width, const int height, const int steer_max_angle, const int detect_line_count)
  : LaneDetector(width, height, steer_max_angle, detect_line_count)
{}

void InToOutLaneDetector::setHsvSBinThres(const int bin_thres) { hsv_s_bin_thres_ = bin_thres; }
void InToOutLaneDetector::setLeftDetectOffset(const int offset) { left_detect_offset_ = offset; }
void InToOutLaneDetector::setRightDetectOffset(const int offset) { right_detect_offset_ = offset; }

int InToOutLaneDetector::getHsvSBinThres() const { return hsv_s_bin_thres_; }
int InToOutLaneDetector::getLeftDetectOffset() const { return left_detect_offset_; }
int InToOutLaneDetector::getRightDetectOffset() const { return right_detect_offset_; }
void InToOutLaneDetector::getRoiGrayBinImg(Mat& img) { img = roi_bin_img_from_gray_; }
void InToOutLaneDetector::getRoiHsvSBinImg(Mat& img) { img = roi_bin_img_from_hsv_s_; }

void InToOutLaneDetector::cvtToRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size)
{
    Mat roi_hsv_img;
    Mat roi_gray_img;

    // 1. convert roi to hsv
    Mat roi_color_img = resized_img_(Rect(left_top.x, left_top.y, roi_size.width, roi_size.height));
    cvtColor(roi_color_img, roi_gray_img, COLOR_BGR2GRAY);
    cvtColor(roi_color_img, roi_hsv_img, COLOR_BGR2HSV);

    // 2. extract 's' image from hsv image
    Mat roi_hsv_s_img;
    vector<Mat> hsv_planes;
    split(roi_hsv_img, hsv_planes);
    roi_hsv_s_img = hsv_planes[1];  // s image

    // 3. convert 's' image and gray to binary
    threshold(roi_gray_img, roi_bin_img_from_gray_, gray_bin_thres_, 255, THRESH_BINARY);
    threshold(roi_hsv_s_img, roi_bin_img_from_hsv_s_, hsv_s_bin_thres_, 255, THRESH_BINARY);

    // 4. add these two binary images
    roi_binary_img_ = roi_bin_img_from_gray_ + roi_bin_img_from_hsv_s_;
}

void InToOutLaneDetector::resetLeftPoint(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  last_left_point_arr_[index].x = line_point_detector_arr_[index].find_L0_x_in2out(roi_binary_img_, detect_y_offset_arr_[index], last_left_point_arr_[index].x, left_detect_offset_);
  last_left_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
}

void InToOutLaneDetector::resetRightPoint(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  last_right_point_arr_[index].x = line_point_detector_arr_[index].find_R0_x_in2out(roi_binary_img_, detect_y_offset_arr_[index], last_right_point_arr_[index].x, right_detect_offset_);
  last_right_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
}

void InToOutLaneDetector::updateNextPoint(const int index)
  throw(my_out_of_range)
{
  if(index >= DETECT_LINE_COUNT_) {
    throw_my_out_of_range(getOutOfRangeMsg(index, DETECT_LINE_COUNT_));
  }

  cur_right_point_arr_[index].x = line_point_detector_arr_[index].find_RN_x_in2out(roi_binary_img_, last_right_point_arr_[index].x, detect_y_offset_arr_[index], LINE_PIXEL_THRESHOLD, right_detect_offset_);
	cur_right_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
	last_right_point_arr_[index].x = cur_right_point_arr_[index].x;

	cur_left_point_arr_[index].x = line_point_detector_arr_[index].find_LN_x_in2out(roi_binary_img_, last_left_point_arr_[index].x, detect_y_offset_arr_[index], LINE_PIXEL_THRESHOLD, left_detect_offset_);
	cur_left_point_arr_[index].y = roi_binary_img_.rows * detect_y_offset_arr_[index] / 100;
	last_left_point_arr_[index].x = cur_left_point_arr_[index].x;
}

void InToOutLaneDetector::showImg() const
{
  LaneDetector::showImg();
  imshow("binary from gray", roi_bin_img_from_gray_);
	imshow("binary from hsv s", roi_bin_img_from_hsv_s_);
	waitKey(3);
}
