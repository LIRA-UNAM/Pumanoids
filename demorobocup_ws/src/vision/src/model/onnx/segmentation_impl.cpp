#include "booster_vision/model/onnx/segmentation_impl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "onnx_common.hpp"

namespace booster_vision {

YoloV8SegmentorONNX::~YoloV8SegmentorONNX() {
    onnx_detail::FreeBuffer(data_buffer_, element_type_);
}

void YoloV8SegmentorONNX::Init(std::string model_path) {
    onnx_detail::ValidateModelFileExists(model_path, "Segmentation");
    try {
        env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "YoloSeg");
        Ort::SessionOptions session_option;
        onnx_detail::ConfigureSessionProvider(session_option, "Segmentation");
        std::cout << "[Segmentation] Loading ONNX model: " << model_path << std::endl;

        session_ = std::make_shared<Ort::Session>(env_, model_path.c_str(), session_option);
        onnx_detail::AllocateBufferAndCollectIO(
            *session_,
            "Segmentation",
            model_input_size_,
            element_type_,
            data_buffer_,
            input_name_storage_,
            output_name_storage_,
            input_node_names_,
            output_node_names_);

        if (session_->GetOutputCount() < 2) {
            throw std::runtime_error(
                "segmentation onnx model must provide 2 outputs (det + proto), got " +
                std::to_string(session_->GetOutputCount()));
        }

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
        std::cout << "[Segmentation] warmup takes: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
                  << "ms" << std::endl;
    } catch (const std::exception &e) {
        throw std::runtime_error("segmentation onnx model init failed: " + std::string(e.what()));
    }
}

template <typename T>
std::vector<booster_vision::SegmentationRes> YoloV8SegmentorONNX::InferenceImpl(const cv::Mat &img) {
    const int img_width = img.cols;
    const int img_height = img.rows;

    Ort::Value input_tensor = onnx_detail::PrepareInputTensor<T>(img, model_input_size_, data_buffer_);

    auto start = std::chrono::high_resolution_clock::now();
    auto output_tensors = session_->Run(
        options_,
        input_node_names_.data(),
        &input_tensor,
        1,
        output_node_names_.data(),
        output_node_names_.size());
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "segmentation inference takes: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();

    Ort::TypeInfo det_type_info = output_tensors[0].GetTypeInfo();
    auto det_info = det_type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> det_dims = det_info.GetShape();
    auto *det_output = output_tensors[0].GetTensorMutableData<T>();

    constexpr int kMaskDims = 32;
    const int num_channels = static_cast<int>(det_dims[1]);
    const int num_anchors = static_cast<int>(det_dims[2]);
    const int num_classes = num_channels - 4 - kMaskDims;
    if (num_classes <= 0) {
        throw std::runtime_error("unexpected segmentation output channel count: " + std::to_string(num_channels));
    }

    Ort::TypeInfo proto_type_info = output_tensors[1].GetTypeInfo();
    auto proto_info = proto_type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> proto_dims = proto_info.GetShape();
    auto *proto_output = output_tensors[1].GetTensorMutableData<T>();
    const int mask_height = static_cast<int>(proto_dims[2]);
    const int mask_width = static_cast<int>(proto_dims[3]);

    cv::Mat raw_data;
    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        raw_data = cv::Mat(num_channels, num_anchors, CV_16F, det_output);
        raw_data.convertTo(raw_data, CV_32F);
    } else {
        raw_data = cv::Mat(num_channels, num_anchors, CV_32F, det_output);
    }
    raw_data = raw_data.t();

    cv::Mat proto_data;
    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        proto_data = cv::Mat(kMaskDims, mask_height * mask_width, CV_16F, proto_output);
        proto_data.convertTo(proto_data, CV_32F);
    } else {
        proto_data = cv::Mat(kMaskDims, mask_height * mask_width, CV_32F, proto_output);
    }

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<float>> mask_coeffs;

    auto *data = reinterpret_cast<float *>(raw_data.data);
    const float resize_scale =
        (img_width >= img_height)
            ? (img_width / static_cast<float>(model_input_size_.width))
            : (img_height / static_cast<float>(model_input_size_.height));

    for (int i = 0; i < num_anchors; ++i) {
        float *row = data + i * num_channels;
        float *class_scores = row + 4;
        cv::Mat scores(1, num_classes, CV_32FC1, class_scores);
        cv::Point class_id;
        double max_class_score = 0.0;
        cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id);

        if (max_class_score > confidence_) {
            const float x = row[0];
            const float y = row[1];
            const float w = row[2];
            const float h = row[3];

            int left = std::max(0, std::min(static_cast<int>((x - 0.5f * w) * resize_scale), img_width - 1));
            int top = std::max(0, std::min(static_cast<int>((y - 0.5f * h) * resize_scale), img_height - 1));
            int right = std::min(img_width, static_cast<int>((x + 0.5f * w) * resize_scale));
            int bottom = std::min(img_height, static_cast<int>((y + 0.5f * h) * resize_scale));
            int width = right - left;
            int height = bottom - top;

            if (width >= 3 && height >= 3) {
                boxes.emplace_back(left, top, width, height);
                confidences.push_back(static_cast<float>(max_class_score));
                // Single-class line segmentation models (e.g. sim_data_seg_0126.onnx) emit 1 score
                // representing field lines (class_id 1 in kClassLabels = {"CircleLine", "Line"}).
                const int mapped_class = (num_classes == 1 && YoloV8Segmentor::kClassLabels.size() > 1)
                    ? 1
                    : class_id.x;
                class_ids.push_back(mapped_class);
                mask_coeffs.emplace_back(row + 4 + num_classes, row + 4 + num_classes + kMaskDims);
            }
        }
    }

    std::vector<int> nms_results;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_, nms_threshold_, nms_results);

    std::vector<booster_vision::SegmentationRes> results;
    results.reserve(nms_results.size());
    const auto *proto_ptr = reinterpret_cast<const float *>(proto_data.data);

    for (int idx : nms_results) {
        booster_vision::SegmentationRes result;
        result.bbox = boxes[idx];
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.class_name = YoloV8Segmentor::kClassLabels[class_ids[idx]];

        cv::Mat mask_mat(mask_height, mask_width, CV_32FC1);
        for (int y = 0; y < mask_height; ++y) {
            for (int x = 0; x < mask_width; ++x) {
                float val = 0.0f;
                for (int c = 0; c < kMaskDims; ++c) {
                    val += mask_coeffs[idx][c] * proto_ptr[c * mask_height * mask_width + y * mask_width + x];
                }
                mask_mat.at<float>(y, x) = 1.0f / (1.0f + std::exp(-val));
            }
        }

        cv::resize(mask_mat, mask_mat, model_input_size_, 0, 0, cv::INTER_LINEAR);

        int mask_left = std::max(0, std::min(static_cast<int>(boxes[idx].x / resize_scale), model_input_size_.width - 1));
        int mask_top = std::max(0, std::min(static_cast<int>(boxes[idx].y / resize_scale), model_input_size_.height - 1));
        int mask_right = std::min(model_input_size_.width, static_cast<int>((boxes[idx].x + boxes[idx].width) / resize_scale));
        int mask_bottom = std::min(model_input_size_.height, static_cast<int>((boxes[idx].y + boxes[idx].height) / resize_scale));

        cv::Rect mask_roi(mask_left, mask_top, mask_right - mask_left, mask_bottom - mask_top);
        result.mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
        if (mask_roi.width > 0 && mask_roi.height > 0) {
            cv::Mat cropped_mask = mask_mat(mask_roi);
            cv::Mat final_mask;
            cv::resize(cropped_mask, final_mask, boxes[idx].size(), 0, 0, cv::INTER_LINEAR);

            cv::Mat binary_mask;
            cv::threshold(final_mask, binary_mask, 0.5, 255, cv::THRESH_BINARY);
            binary_mask.convertTo(binary_mask, CV_8UC1);
            binary_mask.copyTo(result.mask(boxes[idx]));

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            for (auto &contour : contours) {
                for (auto &pt : contour) {
                    pt.x += boxes[idx].x;
                    pt.y += boxes[idx].y;
                }
            }
            result.contour = std::move(contours);
        }
        results.push_back(std::move(result));
    }

    end = std::chrono::high_resolution_clock::now();
    std::cout << "segmentation post process takes: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "ms" << std::endl;
    return results;
}

std::vector<booster_vision::SegmentationRes> YoloV8SegmentorONNX::Inference(const cv::Mat &img) {
    if (element_type_ == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        return InferenceImpl<float>(img);
    }
    return InferenceImpl<Ort::Float16_t>(img);
}

} // namespace booster_vision
