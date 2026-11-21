#include "booster_vision/model/onnx/detection_impl.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include "onnx_common.hpp"

namespace booster_vision {

YoloV8DetectorONNX::~YoloV8DetectorONNX() {
    onnx_detail::FreeBuffer(data_buffer_, element_type_);
}

void YoloV8DetectorONNX::Init(std::string model_path) {
    onnx_detail::ValidateModelFileExists(model_path, "Detection");
    try {
        env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "Yolo");
        Ort::SessionOptions session_option;
        onnx_detail::ConfigureSessionProvider(session_option, "Detection");
        std::cout << "[Detection] Loading ONNX model: " << model_path << std::endl;

        session_ = std::make_shared<Ort::Session>(env_, model_path.c_str(), session_option);
        onnx_detail::AllocateBufferAndCollectIO(
            *session_,
            "Detection",
            model_input_size_,
            element_type_,
            data_buffer_,
            input_name_storage_,
            output_name_storage_,
            input_node_names_,
            output_node_names_);

        options_ = Ort::RunOptions{nullptr};

        auto start = std::chrono::high_resolution_clock::now();
        cv::Mat warmup_img = cv::Mat::zeros(model_input_size_, CV_8UC3);
        Ort::Value input_tensor = (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            ? onnx_detail::PrepareInputTensor<float>(warmup_img, model_input_size_, data_buffer_)
            : onnx_detail::PrepareInputTensor<Ort::Float16_t>(warmup_img, model_input_size_, data_buffer_);
        for (int i = 0; i < 5; ++i) {
            session_->Run(
                options_,
                input_node_names_.data(),
                &input_tensor,
                1,
                output_node_names_.data(),
                output_node_names_.size());
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "[Detection] warmup takes: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
    } catch (const std::exception &e) {
        throw std::runtime_error("detection onnx model init failed: " + std::string(e.what()));
    }
}

template <typename T>
std::vector<booster_vision::DetectionRes> YoloV8DetectorONNX::InferenceImpl(const cv::Mat &img) {
    const int img_width = img.cols;
    const int img_height = img.rows;
    Ort::Value input_tensor = onnx_detail::PrepareInputTensor<T>(img, model_input_size_, data_buffer_);

    auto start = std::chrono::high_resolution_clock::now();
    auto output_tensor = session_->Run(
        options_,
        input_node_names_.data(),
        &input_tensor,
        1,
        output_node_names_.data(),
        output_node_names_.size());
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "inference takes: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    Ort::TypeInfo type_info = output_tensor.front().GetTypeInfo();
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> output_node_dims = tensor_info.GetShape();
    auto *output = output_tensor.front().GetTensorMutableData<T>();

    const int signal_res_num = static_cast<int>(output_node_dims[1]);
    const int stride_num = static_cast<int>(output_node_dims[2]);
    const int num_classes = std::min(
        signal_res_num - 4,
        static_cast<int>(YoloV8Detector::kClassLabels.size()));
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    cv::Mat raw_data;

    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        raw_data = cv::Mat(signal_res_num, stride_num, CV_16F, output);
        raw_data.convertTo(raw_data, CV_32F);
    } else {
        raw_data = cv::Mat(signal_res_num, stride_num, CV_32F, output);
    }
    raw_data = raw_data.t();

    auto *data = reinterpret_cast<float *>(raw_data.data);
    const float resize_scales =
        (img_width >= img_height)
            ? (img_width / static_cast<float>(model_input_size_.width))
            : (img_height / static_cast<float>(model_input_size_.height));

    for (int i = 0; i < stride_num; ++i) {
        float *class_scores = data + 4;
        cv::Mat scores(1, num_classes, CV_32FC1, class_scores);
        cv::Point class_id;
        double max_class_score = 0.0;
        cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id);
        if (max_class_score > confidence_) {
            const float x = data[0];
            const float y = data[1];
            const float w = data[2];
            const float h = data[3];

            int left = static_cast<int>((x - 0.5f * w) * resize_scales);
            int top = static_cast<int>((y - 0.5f * h) * resize_scales);
            int width = static_cast<int>(w * resize_scales);
            int height = static_cast<int>(h * resize_scales);

            if (left >= 0 && left < img_width && top >= 0 && top < img_height) {
                const int right = std::min(img_width - 1, left + width);
                const int bottom = std::min(img_height - 1, top + height);
                width = right - left;
                height = bottom - top;
                if (width >= 3 && height >= 3) {
                    boxes.emplace_back(left, top, width, height);
                    confidences.push_back(static_cast<float>(max_class_score));
                    class_ids.push_back(class_id.x);
                }
            }
        }
        data += signal_res_num;
    }

    std::vector<int> nms_results;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_, nms_threshold_, nms_results);
    std::vector<booster_vision::DetectionRes> ret;
    ret.reserve(nms_results.size());
    for (int idx : nms_results) {
        booster_vision::DetectionRes result;
        result.bbox = boxes[idx];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.class_name = YoloV8Detector::kClassLabels[class_ids[idx]];
        ret.push_back(result);
    }
    end = std::chrono::high_resolution_clock::now();
    std::cout << "post process takes: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms" << std::endl;
    return ret;
}

std::vector<booster_vision::DetectionRes> YoloV8DetectorONNX::Inference(const cv::Mat &img) {
    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        return InferenceImpl<float>(img);
    }
    return InferenceImpl<Ort::Float16_t>(img);
}

} // namespace booster_vision
