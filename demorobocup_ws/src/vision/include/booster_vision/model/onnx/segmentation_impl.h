#pragma once

#include "booster_vision/model/segmentor.h"

#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

namespace booster_vision {

class YoloV8SegmentorONNX : public booster_vision::YoloV8Segmentor {
public:
    YoloV8SegmentorONNX(const std::string &path, const float &conf) :
        booster_vision::YoloV8Segmentor(path, conf) {
        Init(path);
    }
    ~YoloV8SegmentorONNX() override;

    void Init(std::string model_path);
    std::vector<booster_vision::SegmentationRes> Inference(const cv::Mat &img) override;

private:
    template <typename T>
    std::vector<booster_vision::SegmentationRes> InferenceImpl(const cv::Mat &img);

    Ort::Env env_{nullptr};
    std::shared_ptr<Ort::Session> session_;
    Ort::RunOptions options_{nullptr};
    std::vector<std::string> input_name_storage_;
    std::vector<std::string> output_name_storage_;
    std::vector<const char *> input_node_names_;
    std::vector<const char *> output_node_names_;
    ONNXTensorElementDataType element_type_ = ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT;
    void *data_buffer_ = nullptr;

    cv::Size model_input_size_ = {640, 640};
};

} // namespace booster_vision
