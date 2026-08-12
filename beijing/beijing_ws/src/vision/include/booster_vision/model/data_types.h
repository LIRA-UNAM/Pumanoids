#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace booster_vision {

struct DetectionRes {
    cv::Rect bbox;
    int class_id;
    std::string class_name;
    float confidence;
};

struct SegmentationRes {
    cv::Mat mask;
    std::vector<std::vector<cv::Point>> contour;
    cv::Rect bbox;
    int class_id;
    std::string class_name;
    float confidence;
};

} // namespace booster_vision