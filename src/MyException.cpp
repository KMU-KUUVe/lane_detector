#include "lane_detector/MyException.h"

std::string getOutOfRangeMsg(const int index, const int detect_line_count) { return "passed index: " + std::to_string(index) + ". it must be less than detect_line_count: " + std::to_string(detect_line_count); }
