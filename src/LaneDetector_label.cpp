#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "lane_detector/LaneDetector.h"

using namespace std;
using namespace cv;

//ȭ�� resize

#define resize_n 1
#define LINE_LENGTH 30

// ���Ⱒ �Ǵ� ��ġ y
#define steer_height 70

#define PI 3.141592

//���� ���� ����  
Scalar lower_white = Scalar(200, 200, 200); //��� ���� (RGB)
Scalar upper_white = Scalar(255, 255, 255);
Scalar lower_yellow = Scalar(10, 100, 100); //����� ���� (HSV)
Scalar upper_yellow = Scalar(40, 255, 255);




// ����þ� ��, �ŵ�� ���� ���
cv::Mat LaneDetector::deNoise(cv::Mat inputImage) {
  cv::Mat output;

  cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);     // ����þ� ��
//  medianBlur(inputImage, output, 3);    // �̵�� �� -> ���� ���� ������.

  return output;
}




// ���� ���� �Լ�. 
cv::Mat LaneDetector::edgeDetector(cv::Mat img_noise) {
  cv::Mat output;
  cv::Mat kernel;
  cv::Point anchor;

  // Convert image from RGB to gray
  cv::cvtColor(img_noise, output, cv::COLOR_RGB2GRAY);
  // Binarize gray image
  cv::threshold(output, output, 120, 255, cv::THRESH_BINARY);
  imshow("binary �̹���", output);


  // Create the kernel [-1 0 1] -> ���ι��� ���� �����̴�.
  // This kernel is based on the one found in the
  // Lane Departure Warning System by Mathworks
  
  anchor = cv::Point(-1, -1);
  kernel = cv::Mat(1, 3, CV_32F);
  kernel.at<float>(0, 0) = -1;
  kernel.at<float>(0, 1) = 0;
  kernel.at<float>(0, 2) = 1;

  // Filter the binary image to obtain the edges
  cv::filter2D(output, output, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);

  imshow("���� �̹���", output);
  return output;
}

// MASK THE EDGE IMAGE
/**
 *@brief Mask the image so that only the edges that form part of the lane are detected
 *@param img_edges is the edges image from the previous function
 *@return Binary image with only the desired edges being represented
 */

 /* method
 0 : ���� ���ϴ� ����ũ ��� �����
 1 :
 */
cv::Mat LaneDetector::mask(cv::Mat frame, int method) {
  cv::Mat output;
  
  // ���� ���ϴ� ����ũ�� �����Ѵ�.
  if (method == 0)
  {
	  cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());

	  // �ð�������� ����Ʈ�� �����Ѵ�.
	  // Point(x,y)
	  // TODO : �Ʒ� 4���� ������ Ư�� ���� �ƴ� ������ ����
	  /*
	  cv::Point pts[4] = {
		  cv::Point(210/ resize_n, 720/ resize_n),
		  cv::Point(550/ resize_n, 450/ resize_n),
		  cv::Point(716/ resize_n, 450/ resize_n),
		  cv::Point(1280/ resize_n, 720/ resize_n)
	  };
	 */
	  
	  cv::Point pts[4] = {
		  cv::Point(0, frame.rows),
		  cv::Point(0, frame.rows/2),
		  cv::Point(frame.cols, frame.rows / 2),
		  cv::Point(frame.cols, frame.rows)
	  };
	  // Create a binary polygon mask
	  cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));
	  // Multiply the edges image and the mask to get the output
	  cv::bitwise_and(frame, mask, output);

	  return output;
  }
  else if (method == 1)  //  ȭ���� ���� �ڸ���.
  {
	  return frame(Rect(0, frame.rows / 2, frame.cols , frame.rows / 2));
  }
}

// HOUGH LINES
/**
 *@brief Obtain all the line segments in the masked images which are going to be part of the lane boundaries
 *@param img_mask is the masked binary image from the previous function
 *@return Vector that contains all the detected lines in the image
 */
std::vector<cv::Vec4i> LaneDetector::houghLines(cv::Mat img_mask) {
  std::vector<cv::Vec4i> line;

  // rho and theta are selected by trial and error
  // input, output, rho, theta, thre,...)
  HoughLinesP(img_mask, line, 1, CV_PI/180, LINE_LENGTH, 20 , 30);

  return line;
}

// SORT RIGHT AND LEFT LINES
/**
 *@brief Sort all the detected Hough lines by slope.
 *@brief The lines are classified into right or left depending
 *@brief on the sign of their slope and their approximate location
 *@param lines is the vector that contains all the detected lines
 *@param img_edges is used for determining the image center
 *@return The output is a vector(2) that contains all the classified lines
 */
std::vector<std::vector<cv::Vec4i> > LaneDetector::lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
  std::vector<std::vector<cv::Vec4i> > output(2);
  size_t j = 0;
  cv::Point ini;
  cv::Point fini;
  double slope_thresh = 0.3;
  std::vector<double> slopes;
  std::vector<cv::Vec4i> selected_lines;
  std::vector<cv::Vec4i> right_lines, left_lines;

  // Calculate the slope of all the detected lines
  for (auto i : lines) {
    ini = cv::Point(i[0], i[1]);
    fini = cv::Point(i[2], i[3]);

    // Basic algebra: slope = (y1 - y0)/(x1 - x0)
    double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y))/(static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

    // If the slope is too horizontal, discard the line
    // If not, save them  and their respective slope
    if (std::abs(slope) > slope_thresh) {
      slopes.push_back(slope);
      selected_lines.push_back(i);
    }
  }

  // Split the lines into right and left lines
  img_center = static_cast<double>((img_edges.cols / 2));
  while (j < selected_lines.size()) {
    ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
    fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);

    // Condition to classify line as left side or right side
    if (slopes[j] > 0 && fini.x > img_center && ini.x > img_center) {
      right_lines.push_back(selected_lines[j]);
      right_flag = true;
    } else if (slopes[j] < 0 && fini.x < img_center && ini.x < img_center) {
        left_lines.push_back(selected_lines[j]);
        left_flag = true;
    }
    j++;
  }

  output[0] = right_lines;
  output[1] = left_lines;
  
  return output;
}

// REGRESSION FOR LEFT AND RIGHT LINES
/**
 *@brief Regression takes all the classified line segments initial and final points and fits a new lines out of them using the method of least squares.
 *@brief This is done for both sides, left and right.
 *@param left_right_lines is the output of the lineSeparation function
 *@param inputImage is used to select where do the lines will end
 *@return output contains the initial and final points of both lane boundary lines
 */

std::vector<cv::Point> LaneDetector::regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage, double &angle) {
  std::vector<cv::Point> output(4);
  cv::Point ini;
  cv::Point fini;
  cv::Point ini2;
  cv::Point fini2;
  cv::Vec4d right_line;
  cv::Vec4d left_line;
  Point line_middle;
  std::vector<cv::Point> right_pts;
  std::vector<cv::Point> left_pts;

  // If right lines are being detected, fit a line using all the init and final points of the lines
  
  if (right_flag == true) {
    for (auto i : left_right_lines[0]) {
      ini = cv::Point(i[0], i[1]);
      fini = cv::Point(i[2], i[3]);

      right_pts.push_back(ini);
      right_pts.push_back(fini);
    }
	
    if (right_pts.size() > 0) {
      // The right line is formed here
      cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
      right_m = right_line[1] / right_line[0];
      right_b = cv::Point(right_line[2], right_line[3]);
    }
  }

  // If left lines are being detected, fit a line using all the init and final points of the lines
  if (left_flag == true) {
    for (auto j : left_right_lines[1]) {
      ini2 = cv::Point(j[0], j[1]);
      fini2 = cv::Point(j[2], j[3]);

      left_pts.push_back(ini2);
      left_pts.push_back(fini2);
    }

    if (left_pts.size() > 0) {
      // The left line is formed here
      cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
      left_m = left_line[1] / left_line[0];
      left_b = cv::Point(left_line[2], left_line[3]);
    }
  }

  // One the slope and offset points have been obtained, apply the line equation to obtain the line points
  int ini_y = inputImage.rows;
  int fin_y = 470/ resize_n;

  double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
  double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

  double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
  double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

  // ���Ⱒ  ����ϱ�.

  line_middle.y = steer_height / 100.0 * inputImage.rows;

  int left_xx = (line_middle.y - left_b.y) / (left_m)+left_b.x;
  int right_xx = (line_middle.y - right_b.y) / (right_m)+right_b.x;

  line_middle.x = (right_xx + left_xx)/2.0;

  circle(inputImage, line_middle , 5, Scalar(255, 255, 0), 5);
 
  angle = atan2(line_middle.x - inputImage.cols / 2.0, inputImage.rows - line_middle.y) * 180 / PI;

  //

  output[0] = cv::Point(right_ini_x, ini_y);
  output[1] = cv::Point(right_fin_x, fin_y);
  output[2] = cv::Point(left_ini_x, ini_y);
  output[3] = cv::Point(left_fin_x, fin_y);

  return output;
}

// TURN PREDICTION
/**
 *@brief Predict if the lane is turning left, right or if it is going straight
 *@brief It is done by seeing where the vanishing point is with respect to the center of the image
 *@return String that says if there is left or right turn or if the road is straight
 */

std::string LaneDetector::predictTurn() {
  std::string output;
  double vanish_x;
  double thr_vp = 10/ resize_n;

  // The vanishing point is the point where both lane boundary lines intersect
  vanish_x = static_cast<double>(((right_m*right_b.x) - (left_m*left_b.x) - right_b.y + left_b.y) / (right_m - left_m));

  // The vanishing points location determines where is the road turning
  if (vanish_x < (img_center - thr_vp))
    output = "Left Turn";
  else if (vanish_x > (img_center + thr_vp))
    output = "Right Turn";
  else if (vanish_x >= (img_center - thr_vp) && vanish_x <= (img_center + thr_vp))
    output = "Straight";

  return output;
}

// PLOT RESULTS
/**
 *@brief This function plots both sides of the lane, the turn prediction message and a transparent polygon that covers the area inside the lane boundaries
 *@param inputImage is the original captured frame
 *@param lane is the vector containing the information of both lines
 *@param turn is the output string containing the turn information
 *@return The function returns a 0
 */



int LaneDetector::plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn) {
  std::vector<cv::Point> poly_points;
  cv::Mat output;

  // Create the transparent polygon for a better visualization of the lane
  
  inputImage.copyTo(output);
  poly_points.push_back(lane[2]);
  poly_points.push_back(lane[0]);
  poly_points.push_back(lane[1]);
  poly_points.push_back(lane[3]);
  cv::fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);
  cv::addWeighted(output, 0.3, inputImage, 1.0 - 0.3, 0, inputImage);
  


  // Plot both lines of the lane boundary
  cv::line(inputImage, lane[0], lane[1], cv::Scalar(0, 255, 255), 5, CV_AA);
  cv::line(inputImage, lane[2], lane[3], cv::Scalar(0, 255, 255), 5, CV_AA);

  // Plot the turn message
  cv::putText(inputImage, turn, cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
  
  // Show the final output image
  cv::namedWindow("Lane", CV_WINDOW_AUTOSIZE);
  cv::imshow("Lane", inputImage);
  return 0;
}



// �������, ����� ���� �����ϱ�.
void LaneDetector::filter_colors(Mat _img_bgr, Mat &img_filtered)
{
	// Filter the image to include only yellow and white pixels
	Mat img_bgr;
	_img_bgr.copyTo(img_bgr);
	Mat img_hsv, img_combine;
	Mat white_mask, white_image;
	Mat yellow_mask, yellow_image;


	//Filter white pixels
	inRange(img_bgr, lower_white, upper_white, white_mask);
	bitwise_and(img_bgr, img_bgr, white_image, white_mask);


	//Filter yellow pixels( Hue 30 )
	cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);


	inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);
	bitwise_and(img_bgr, img_bgr, yellow_image, yellow_mask);


	//Combine the two above images
	addWeighted(white_image, 1.0, yellow_image, 1.0, 0.0, img_combine);


	img_combine.copyTo(img_filtered);
}


void LaneDetector::DrawLabelingImage(Mat image)
{
	Mat img_gray, img_color, img_binary;

	cvtColor(image, img_gray, COLOR_BGR2GRAY);  // Convert the image to Gray
	threshold(img_gray, img_binary, 127, 255, THRESH_BINARY);
	cvtColor(img_gray, img_color, COLOR_GRAY2BGR);


	Mat img_labels, stats, centroids;
	int numOfLables = connectedComponentsWithStats(img_binary, img_labels, stats, centroids, 8, CV_32S);


	//�󺧸��� �̹����� Ư�� ���� �÷��� ǥ�����ֱ� 
	for (int y = 0; y<img_labels.rows; ++y) {

		int *label = img_labels.ptr<int>(y);
		Vec3b* pixel = img_color.ptr<Vec3b>(y);


		for (int x = 0; x < img_labels.cols; ++x) {


			if (label[x] == 3) {
				pixel[x][2] = 0;
				pixel[x][1] = 255;
				pixel[x][0] = 0;
			}
		}
	}


	//�󺧸� �� �̹����� ���� ���簢������ �ѷ��α� 
	for (int j = 1; j < numOfLables; j++) {
		int area = stats.at<int>(j, CC_STAT_AREA);
		int left = stats.at<int>(j, CC_STAT_LEFT);
		int top = stats.at<int>(j, CC_STAT_TOP);
		int width = stats.at<int>(j, CC_STAT_WIDTH);
		int height = stats.at<int>(j, CC_STAT_HEIGHT);

		int x = centroids.at<double>(j, 0); //�߽���ǥ
		int y = centroids.at<double>(j, 1);

		circle(img_color, Point(x, y), 5, Scalar(255, 0, 0), 1);

		rectangle(img_color, Point(left, top), Point(left + width, top + height),
			Scalar(0, 0, 255), 1);

		putText(img_color, to_string(j), Point(left + 20, top + 20), FONT_HERSHEY_SIMPLEX,
			1, Scalar(255, 0, 0), 2);
	}

	namedWindow("Labeling Image", WINDOW_AUTOSIZE);             // Create a window for display
	imshow("Labeling Image", img_color);                        // Show our image inside it
}
