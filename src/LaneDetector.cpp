#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "lane_detector/LaneDetector.h"


using namespace std;
using namespace cv;

#define PI 3.141592

//bluring the images for remove noise.
cv::Mat LaneDetector::deNoise(cv::Mat inputImage) {
	cv::Mat output;
	cv::GaussianBlur(inputImage, output, cv::Size(3, 3), 0, 0);
	//  medianBlur(inputImage, output, 3);
	return output;
}


// extract white and yellow lane.
void LaneDetector::filter_colors(Mat _img_bgr, Mat &img_filtered)
{
	lower_white_rgb = Scalar(WITHE_RGB_THRES, WITHE_RGB_THRES, WITHE_RGB_THRES); //(RGB)
	upper_white_rgb = Scalar(255, 255, 255);
	lower_yellow_hsv = Scalar(10, 180, 180); // (HSV)
	upper_yellow_hsv = Scalar(40, 255, 255);
	lower_white_hsv = Scalar(0, 0, 180); //(HSV)
	upper_white_hsv = Scalar(360, 65, 255);
	// Filter the image to include only yellow and white pixels
	Mat img_bgr;
	_img_bgr.copyTo(img_bgr);
	Mat test;
	_img_bgr.copyTo(test);
	Mat img_hsv, img_combine;
	Mat white_mask_rgb, white_image_rgb;
	Mat yellow_mask, yellow_image;
	Mat white_mask_hsv, white_image_hsv;
	Mat thre, grayy;
	
	cvtColor(img_bgr, grayy, COLOR_BGR2GRAY);
	cv::threshold(img_bgr, thre, 180, 255, cv::THRESH_BINARY);
//	imshow("thre....", thre);

	/*
	//Filter white pixels with RGB
	inRange(img_bgr, lower_white_rgb, upper_white_rgb, white_mask_rgb);
	bitwise_and(img_bgr, img_bgr, white_image_rgb, white_mask_rgb);
	imshow("white rgb", white_image_rgb);
	*/	
	
	 //using white - hsv filtering
	   cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);
	   inRange(img_hsv, lower_white_hsv, upper_white_hsv, white_mask_hsv);
	   bitwise_and(img_bgr, img_bgr, white_image_hsv, white_mask_hsv);

	   cv::threshold(white_image_hsv, white_image_hsv, 170, 255, cv::THRESH_BINARY);
	   imshow("white_hsv", white_image_hsv);
	
	/*
	//Filter yellow pixels( Hue 30 )
	cvtColor(img_bgr, img_hsv, COLOR_BGR2HSV);
	inRange(img_hsv, lower_yellow_hsv, upper_yellow_hsv, yellow_mask);
	bitwise_and(img_bgr, img_bgr, yellow_image, yellow_mask);
	*/
	//Combine the two above images
	//addWeighted(white_image_rgb, 1.0, yellow_image, 1.0, 0.0, img_combine);
	
	//using white - hsv filtering
	//addWeighted(white_image_hsv, 1.0, yellow_image, 1.0, 0.0, img_combine);
	white_image_hsv.copyTo(img_filtered);


}


//edge detection
cv::Mat LaneDetector::edgeDetector(cv::Mat img_noise) {
	cv::Mat output;
	cv::Mat kernel;
	cv::Point anchor;

	// Convert image from RGB to gray
	cv::cvtColor(img_noise, output, cv::COLOR_RGB2GRAY);
	// Binarize gray image
	cv::threshold(output, output, 120, 255, cv::THRESH_BINARY);
	//imshow("binary_gray", output);

	// Create the kernel [-1 0 1]
	// This kernel is based on the one found in the
	// Lane Departure Warning System by Mathworks
	anchor = cv::Point(-1, -1);
	kernel = cv::Mat(1, 3, CV_32F);
	kernel.at<float>(0, 0) = -1;
	kernel.at<float>(0, 1) = 0;
	kernel.at<float>(0, 2) = 1;

	// Filter the binary image to obtain the edges
	cv::filter2D(output, output, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);

	//imshow("filter2d", output);
	return output;
}

// MASK THE EDGE IMAGE
/**
 *@brief Mask the image so that only the edges that form part of the lane are detected
 *@param img_edges is the edges image from the previous function
 *@return Binary image with only the desired edges being represented
 */

/* method
0 : hexagon ROI
1 : rectangle ROI
 */
cv::Mat LaneDetector::mask(cv::Mat frame, int method) {
	cv::Mat output;

	if (method == 0)
	{
		cv::Mat mask = cv::Mat::zeros(frame.size(), frame.type());

		cv::Point pts[6] = {
			cv::Point(frame.cols*x3, frame.rows*height3),
			cv::Point(frame.cols*x2, frame.rows*height2),
			cv::Point(frame.cols*x1, frame.rows*height1),
			cv::Point(frame.cols*(1-x1), frame.rows*height1),
			cv::Point(frame.cols*(1-x2), frame.rows*height2),
			cv::Point(frame.cols*(1-x3), frame.rows*height3)
		};
		/* use trapezoid ROI
		   cv::Point pts[4] = {
		   cv::Point(0, frame.rows),
		   cv::Point(0, frame.rows*0.3),
		   cv::Point(frame.cols, frame.rows*0.3),
		   cv::Point(frame.cols, frame.rows)
		   };
		 */
		// Create a binary polygon mask
		cv::fillConvexPoly(mask, pts, 6, cv::Scalar(255, 0, 0));
		// Multiply the edges image and the mask to get the output
		cv::bitwise_and(frame, mask, output);

		return output;
	}
	else if (method == 1)  //
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
 **/

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
	int ini_y = inputImage.rows;
	int fin_y = inputImage.rows * detect_n ;
	double right_ini_x;
	double right_fin_x;
	double right_xx;
	double left_ini_x;
	double left_fin_x;
	double left_xx;

	line_middle.y = steer_height / 100.0 * inputImage.rows;

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
		right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
		right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;
		right_xx = (line_middle.y - right_b.y) / (right_m)+right_b.x;


	}
	else{
		right_ini_x = inputImage.cols;
		right_fin_x = inputImage.cols;
		right_xx = inputImage.cols;


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
		left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
		left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;
		left_xx = (line_middle.y - left_b.y) / (left_m)+left_b.x;


	}
	else{
		left_ini_x = 0;
		left_fin_x = 0;
		left_xx = 0;
	}

	// One the slope and offset points have been obtained, apply the line equation to obtain the line points
	line_middle.x = (right_xx + left_xx)/2.0;

	circle(inputImage, line_middle , 5, Scalar(255, 255, 0), 5);

	angle = atan2(line_middle.x - inputImage.cols / 2.0, inputImage.rows - line_middle.y) * 180 / PI;
	//angle = -atan2(line_middle.x - inputImage.cols / 2.0, inputImage.rows - line_middle.y) * 180 / PI;

	if(angle >23){
		angle = 23;
	}else if(angle < -23){
		angle = -23;
	}
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
	double thr_vp = 10/resize_n_turn;

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
	int i = 0;
	// Create the transparent polygon for a better visualization of the lane
	/* // for debug
	   for(i=0; i < 4; i++){
	   cout << lane[i] << endl;
	   }*/
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
