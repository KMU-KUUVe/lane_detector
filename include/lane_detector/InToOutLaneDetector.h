#ifndef INTOOUTLANEDETECTOR_H
#define INTOOUTLANEDETECTOR_H

#include "LaneDetector.h"

class InToOutLaneDetector: public LaneDetector
{
public:
  InToOutLaneDetector(const int width, const int height, const int steer_max_angle, const int detect_line_count);

  void setHsvSBinThres(const int bin_thres);
  void setLeftDetectOffset(const int offset);
  void setRightDetectOffset(const int offset);

  int getHsvSBinThres() const;
  int getLeftDetectOffset() const;
  int getRightDetectOffset() const;
  void getRoiGrayBinImg(cv::Mat& img);
  void getRoiHsvSBinImg(cv::Mat& img);

	virtual void cvtToRoiBinaryImg(const cv::Point& left_top, const cv::Size& roi_size);

	virtual void resetLeftPoint(const int index) throw(my_out_of_range);
	virtual void resetRightPoint(const int index) throw(my_out_of_range);

	virtual void updateNextPoint(const int index) throw(my_out_of_range);

  virtual void showImg() const;

protected:
  cv::Mat roi_bin_img_from_gray_;
  cv::Mat roi_bin_img_from_hsv_s_;

  int hsv_s_bin_thres_;

  int left_detect_offset_;
  int right_detect_offset_;
};

#endif
