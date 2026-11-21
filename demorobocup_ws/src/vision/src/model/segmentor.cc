#include "booster_vision/model//segmentor.h"

#include <filesystem>
#include <stdexcept>

#ifdef ENABLE_TENSORRT
#include "booster_vision/model//trt/impl.h"
#endif
#ifdef ENABLE_ONNX
#include "booster_vision/model/onnx/segmentation_impl.h"
#endif

namespace booster_vision {

const std::vector<std::string> YoloV8Segmentor::kClassLabels = {"CircleLine", "Line"};

std::shared_ptr<YoloV8Segmentor> YoloV8Segmentor::CreateYoloV8Segmentor(
    const YAML::Node &node,
    const std::string &model_path_override) {
    std::string model_path = model_path_override.empty() ? node["model_path"].as<std::string>() : model_path_override;
    float conf_thresh = node["confidence_threshold"].as<float>();

    if (model_path.empty() || !std::filesystem::exists(model_path)) {
        throw std::runtime_error("Segmentation model file does not exist: " + model_path);
    }

    std::shared_ptr<YoloV8Segmentor> segmentor_ptr = nullptr;
    const bool is_onnx = (model_path.find(".onnx") != std::string::npos);
    if (is_onnx) {
#ifdef ENABLE_ONNX
        segmentor_ptr = std::shared_ptr<YoloV8Segmentor>(new YoloV8SegmentorONNX(model_path, conf_thresh));
#else
        throw std::runtime_error(
            "ONNX segmentation model requested ('" + model_path + "') but vision was compiled with ENABLE_ONNX=OFF");
#endif
    } else {
#ifdef ENABLE_TENSORRT
        segmentor_ptr = std::shared_ptr<YoloV8Segmentor>(new YoloV8SegmentorTRT(model_path, conf_thresh));
#else
        throw std::runtime_error(
            "TensorRT segmentation model requested ('" + model_path + "') but vision was compiled with ENABLE_TENSORRT=OFF");
#endif
    }

    if (segmentor_ptr && node["nms_threshold"]) {
        segmentor_ptr->setNMSThreshold(node["nms_threshold"].as<float>());
    }
    return segmentor_ptr;
}

cv::Mat YoloV8Segmentor::DrawSegmentation(const cv::Mat &img, const std::vector<SegmentationRes> &segmentations) {
    static std::unordered_map<int, cv::Scalar> class_colors;
    auto get_color_for_class = [&](int class_id) {
        if (class_colors.find(class_id) == class_colors.end()) {
            cv::RNG rng(class_id);
            class_colors[class_id] = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 256));
        }
        return class_colors[class_id];
    };

    cv::Mat img_out;
    cv::cvtColor(img, img_out, cv::COLOR_RGB2BGR);
    for (const auto &seg : segmentations) {
        auto display_color = get_color_for_class(seg.class_id);
        for (int y = seg.bbox.y; y < seg.bbox.y + seg.bbox.height; y++) {
            for (int x = seg.bbox.x; x < seg.bbox.x + seg.bbox.width; x++) {
                if (seg.mask.at<uchar>(y, x) > 127) {
                    for (int c = 0; c < 3; c++) {
                        img_out.at<cv::Vec3b>(y, x)[c] = std::min(255, static_cast<int>(display_color[c]));
                    }
                }
            }
        }
        cv::rectangle(img_out, seg.bbox, display_color, 2);
        cv::putText(img_out, kClassLabels[seg.class_id] + " " + std::to_string(int(seg.confidence * 100)),
                    cv::Point(seg.bbox.x, seg.bbox.y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);
    }
    return img_out;
}

} // namespace booster_vision
