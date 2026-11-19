#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

#include "booster_vision/base/intrin.h"
#include "booster_vision/base/pose.h"

namespace booster_vision {
class BoardDetector {
public:
    BoardDetector(const cv::Size &board_size, const float square_size, const Intrinsics intr) :
        board_size_(board_size), square_size_(square_size), intr_(intr) {
        for (int i = 0; i < board_size_.height; i++) {
            for (int j = 0; j < board_size_.width; j++) {
                board_points_.push_back(cv::Point3f(j * square_size_, i * square_size_, 0));
            }
        }
    }

    Pose getBoardPose();
    std::vector<cv::Point3f> getBoardPoints() { return board_points_; }
    std::vector<cv::Point2f> getBoardUVs() { return board_uvs_; }
    std::vector<cv::Point2f> getBoardUVsSubpixel();
    cv::Mat getBoardMask(const cv::Mat &img);
    bool DetectBoard(const cv::Mat &img);

private:
    cv::Mat gray_;
    cv::Size board_size_;
    float square_size_;
    Intrinsics intr_;
    std::vector<cv::Point3f> board_points_;
    std::vector<cv::Point2f> board_uvs_;
};
} // namespace booster_vision