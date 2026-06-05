#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>

struct Detection {
  std::string class_name;
  float confidence;
  float center_x;
  float center_y;
  float width;
  float height;
};

class YoloDetector {
public:
  YoloDetector(const std::string &engine_path,
               const std::vector<std::string> &class_names);
  ~YoloDetector();

  std::vector<Detection> detect(const cv::Mat &bgr_image);
  std::pair<int, int> getInputSize() const;

private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};
