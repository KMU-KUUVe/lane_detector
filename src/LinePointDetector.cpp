#include "lane_detector/LinePointDetector.h"
#include "lane_detector/ConditionalCompile.h"

using namespace std;
using namespace cv;

//TODO: algorithm comments of function - Jiwon Park 03/18/18

bool LinePointDetector::getLeftResetFlag() const { return left_reset_flag_; }
bool LinePointDetector::getRightResetFlag() const { return right_reset_flag_; }
int LinePointDetector::getContiDetectPixel() const { return continuous_detect_pixel_; }

void LinePointDetector::setLeftResetFlag(const bool left_reset_flag) { left_reset_flag_ = left_reset_flag; }
void LinePointDetector::setRightResetFlag(const bool right_reset_flag) { right_reset_flag_ = right_reset_flag; }
void LinePointDetector::setContiDetectPixel(const int continuous_detect_pixel) { continuous_detect_pixel_ = continuous_detect_pixel; }


bool LinePointDetector::laneIsDiscontinuous(const Mat& binary_img, const int pre_point_x, const int detect_y_offset)
{
	for(int i = 0; i < continuous_detect_pixel_; i++) {
		if(binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, pre_point_x + (i - continuous_detect_pixel_ / 2)) == 255)
			return false;
	}

	return true;
}

/**
 * 왼쪽 끝부터 오른쪽으로 가면서 흰색픽셀을 찾는 알고리즘
 * 결과적으로 왼쪽 차선에 해당하는 흰색 픽셀을 찾게 된다
 *
 */
int LinePointDetector::find_L0_x_out2in(const Mat& binary_img, const int detect_y_offset, const int last_point_x)
{
	int l0_x = last_point_x;

	for (int i = continuous_detect_pixel_ / 2 + 2; i < binary_img.cols - (continuous_detect_pixel_ / 2 + 2); i++)
	{
		if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
		{
			l0_x = i;
			break;
		}
	}
	left_reset_flag_ = false;
	return l0_x;
}

/**
 * 오른쪽 끝부터 왼쪽으로 가면서 흰색픽셀을 찾는 알고리즘
 * 결과적으로 오른쪽 차선에 해당하는 흰색 픽셀을 찾게 된다
 *
 */
int LinePointDetector::find_R0_x_out2in(const Mat& binary_img, const int detect_y_offset, const int last_point_x)
{
	int r0_x = last_point_x;

	for (int i = continuous_detect_pixel_ / 2 + 2; i < binary_img.cols - (continuous_detect_pixel_ / 2 + 2); i++)
	{
		if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, binary_img.cols - i) == 255)
		{
			r0_x = binary_img.cols - i;
			break;
		}
	}
	right_reset_flag_ = false;
	return r0_x;
}

/**
 * 인식한 전 프레임의 왼쪽 차선의 점을 이용하여 새로운 차선의 점을 찾는 알고리즘
 *
 * 그 전 점을 기준으로 주변에 있는 점의 픽셀값을 확인하여, 모두 흰색이면 차선이 연속적으로 존재한다고 할 수 있다
 * 그렇게 판단되면 전 점과 현재 차선 점의 거리가 매우 가깝다고 할 수 있고, 이 논리를 이용해 threshold를 사용하여 노이즈에 해당하는 점을 차선으로 인식하지 않도록 할 수있다
 *
 * 만약 연속적으로 존재하지 않는다고 판단되면 초기점을 찾는 방법으로 차선의 점을 찾을 수 밖에 없다
 *
 */
int LinePointDetector::find_LN_x_out2in(const Mat& binary_img, const int pre_point_x, const int detect_y_offset, const int line_pixel_threshold)
{
	int Left_N_x = pre_point_x;

		// 불연속 선 이라면
	if (laneIsDiscontinuous(binary_img, pre_point_x, detect_y_offset))
	{
		for (int i = continuous_detect_pixel_ / 2 + 2; i < binary_img.cols - (continuous_detect_pixel_ / 2 + 2); i++)
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
			{
				Left_N_x = i;
#if DEBUG
				cout << "out2in 왼쪽 불연속점 입니다" << Left_N_x << endl;
#endif
				break;
			}
		}
	}
	else // 연속선 이라면
	{
		for (int i = continuous_detect_pixel_ / 2 + 2; i < binary_img.cols - (continuous_detect_pixel_ / 2 + 2); i++)
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
			{

				if ((i > pre_point_x + line_pixel_threshold) || (i < pre_point_x - line_pixel_threshold))	continue;

				Left_N_x = i;
				break;
			}
		}
	}


	return Left_N_x;
}

int LinePointDetector::find_RN_x_out2in(const Mat& binary_img, const int pre_point_x, const int detect_y_offset, const int line_pixel_threshold)
{
	int Right_N_x = pre_point_x;

	// 불연속 선 이라면
	if (laneIsDiscontinuous(binary_img, pre_point_x, detect_y_offset))
	{
		for (int i = continuous_detect_pixel_ / 2 + 2; i < binary_img.cols - (continuous_detect_pixel_ / 2 + 2); i++)
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, binary_img.cols - i) == 255)
			{
				Right_N_x = binary_img.cols - i;
#if DEBUG
				cout << "out2in 오른쪽 불연속선 입니다" << Right_N_x << endl;
#endif
				break;
			}
		}
	}
	else // 연속선 이라면
	{
		for (int i = continuous_detect_pixel_ / 2 + 2; i < binary_img.cols - (continuous_detect_pixel_ / 2 + 2); i++)
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, binary_img.cols - i) == 255)
			{
				if ((binary_img.cols - i > pre_point_x + line_pixel_threshold) || (binary_img.cols - i < pre_point_x - line_pixel_threshold))	continue;

				Right_N_x = binary_img.cols - i;
				break;
			}
		}
	}

	return Right_N_x;
}

int LinePointDetector::find_L0_x_in2out(const Mat& binary_img, const int detect_y_offset, const int last_point_x, const int offset)
{
	int l0_x = last_point_x;

	for (int i = binary_img.cols / 2 + offset; i > continuous_detect_pixel_ / 2 + 2; i--)
	{
		if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
		{
			l0_x = i;
			break;
		}
	}
	left_reset_flag_ = false;
	return l0_x;
}

int LinePointDetector::find_R0_x_in2out(const Mat& binary_img, const int detect_y_offset, const int last_point_x, const int offset)
{
	int r0_x = last_point_x;

	for (int i = binary_img.cols / 2 - offset; i < binary_img.cols - (continuous_detect_pixel_ / 2 + 2); i++)
	{
		if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
		{
			r0_x = i;
			break;
		}
	}
	right_reset_flag_ = false;
	return r0_x;
}

int LinePointDetector::find_LN_x_in2out(const Mat& binary_img, const int pre_point_x, const int detect_y_offset, const int line_pixel_threshold, const int offset)
{
	int Left_N_x = pre_point_x;

	// 불연속 선 이라면
	if (laneIsDiscontinuous(binary_img, pre_point_x, detect_y_offset))
	{
		for (int i = binary_img.cols / 2 + offset; i > continuous_detect_pixel_ / 2 + 2; i--)
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
			{
				Left_N_x = i;
#if DEBUG
				cout << "in2out 왼쪽 불연속점 입니다" << Left_N_x << endl;
#endif
				break;
			}
		}
	}
	else // 연속선 이라면
	{
		for (int i = binary_img.cols / 2 + offset; i > continuous_detect_pixel_ / 2 + 2; i--)
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
			{

				if ((i > pre_point_x + line_pixel_threshold) || (i < pre_point_x - line_pixel_threshold))	continue;

				Left_N_x = i;
				break;
			}
		}

	}

	return Left_N_x;
}

int LinePointDetector::find_RN_x_in2out(const Mat& binary_img, const int pre_point_x, const int detect_y_offset, const int line_pixel_threshold, const int offset)
{
	int Right_N_x = pre_point_x;

	// 불연속 선 이라면
	if (laneIsDiscontinuous(binary_img, pre_point_x, detect_y_offset))
	{
		for (int i = binary_img.cols / 2 - offset; i < binary_img.cols - (continuous_detect_pixel_ / 2 + 2); i++)
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
			{
				Right_N_x = i;
#if DEBUG
				cout << "in2out 오른쪽 불연속선 입니다" << Right_N_x << endl;
#endif
				break;
			}
		}
	}
	else // 연속선 이라면
	{
		for (int i = binary_img.cols / 2 - offset; i < binary_img.cols - (continuous_detect_pixel_ / 2 + 2); i++)
		{
			if (binary_img.at<uchar>(binary_img.rows * detect_y_offset / 100, i) == 255)
			{
				if ((i > pre_point_x + line_pixel_threshold) || (i < pre_point_x - line_pixel_threshold))	continue;

				Right_N_x = i;
				break;
			}
		}
	}

	return Right_N_x;
}
