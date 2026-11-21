#pragma once

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

namespace booster_vision::onnx_detail {

inline cv::Mat PreProcessLetterbox(const cv::Mat &src_img, const cv::Size &dst_img_size) {
    cv::Mat dst_img;
    if (src_img.channels() == 3) {
        dst_img = src_img.clone();
        cv::cvtColor(dst_img, dst_img, cv::COLOR_BGR2RGB);
    } else {
        cv::cvtColor(src_img, dst_img, cv::COLOR_GRAY2RGB);
    }

    if (src_img.cols >= src_img.rows) {
        const float scale = src_img.cols / static_cast<float>(dst_img_size.width);
        cv::resize(dst_img, dst_img, cv::Size(dst_img_size.width, static_cast<int>(src_img.rows / scale)));
    } else {
        const float scale = src_img.rows / static_cast<float>(dst_img_size.height);
        cv::resize(dst_img, dst_img, cv::Size(static_cast<int>(src_img.cols / scale), dst_img_size.height));
    }
    cv::Mat padded = cv::Mat::zeros(dst_img_size, CV_8UC3);
    dst_img.copyTo(padded(cv::Rect(0, 0, dst_img.cols, dst_img.rows)));
    return padded;
}

template <typename T>
inline void BlobFromImage(const cv::Mat &img, T *blob) {
    const int channels = img.channels();
    const int img_height = img.rows;
    const int img_width = img.cols;
    for (int c = 0; c < channels; ++c) {
        for (int h = 0; h < img_height; ++h) {
            for (int w = 0; w < img_width; ++w) {
                blob[c * img_width * img_height + h * img_width + w] =
                    static_cast<T>(img.at<cv::Vec3b>(h, w)[c] / 255.0f);
            }
        }
    }
}

template <typename T>
inline Ort::Value PrepareInputTensor(const cv::Mat &img, const cv::Size &input_size, void *data_buffer) {
    cv::Mat processed_img = PreProcessLetterbox(img, input_size);
    std::vector<int64_t> input_node_dims = {1, 3, input_size.height, input_size.width};

    auto *typed_buffer = reinterpret_cast<T *>(data_buffer);
    BlobFromImage(processed_img, typed_buffer);

    return Ort::Value::CreateTensor<T>(
        Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU),
        typed_buffer,
        3 * input_size.area(),
        input_node_dims.data(),
        input_node_dims.size());
}

inline std::string ConfigureSessionProvider(Ort::SessionOptions &session_option, const std::string &tag) {
    session_option.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_option.SetIntraOpNumThreads(1);
    session_option.SetLogSeverityLevel(3);

    const std::vector<std::string> available_providers = Ort::GetAvailableProviders();
    std::cout << "[" << tag << "] Available ONNX Runtime Providers:";
    for (const auto &provider : available_providers) {
        std::cout << " " << provider;
    }
    std::cout << std::endl;

    if (std::find(available_providers.begin(), available_providers.end(), "OpenVINOExecutionProvider") !=
        available_providers.end()) {
        try {
            OrtOpenVINOProviderOptions openvino_options;
            openvino_options.device_type = "GPU_FP32";
            session_option.AppendExecutionProvider_OpenVINO(openvino_options);
            std::cout << "[" << tag << "] Using OpenVINOExecutionProvider (Intel GPU)" << std::endl;
            return "OpenVINOExecutionProvider";
        } catch (const std::exception &e) {
            std::cout << "[" << tag << "] OpenVINO EP failed: " << e.what() << ", falling back..." << std::endl;
        }
    }

    if (std::find(available_providers.begin(), available_providers.end(), "CUDAExecutionProvider") !=
        available_providers.end()) {
        try {
            OrtCUDAProviderOptions cuda_options;
            cuda_options.device_id = 0;
            session_option.AppendExecutionProvider_CUDA(cuda_options);
            std::cout << "[" << tag << "] Using CUDAExecutionProvider" << std::endl;
            return "CUDAExecutionProvider";
        } catch (const std::exception &e) {
            std::cout << "[" << tag << "] CUDA EP failed: " << e.what() << ", falling back to CPU" << std::endl;
        }
    }

    std::cout << "[" << tag << "] Using CPUExecutionProvider" << std::endl;
    return "CPUExecutionProvider";
}

inline void AllocateBufferAndCollectIO(
    Ort::Session &session,
    const std::string &tag,
    cv::Size &model_input_size,
    ONNXTensorElementDataType &element_type,
    void *&data_buffer,
    std::vector<std::string> &input_name_storage,
    std::vector<std::string> &output_name_storage,
    std::vector<const char *> &input_node_names,
    std::vector<const char *> &output_node_names) {
    Ort::TypeInfo input_type_info = session.GetInputTypeInfo(0);
    auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
    const auto input_shape = input_info.GetShape();
    if (input_shape.size() == 4 && input_shape[2] > 0 && input_shape[3] > 0) {
        model_input_size = {static_cast<int>(input_shape[3]), static_cast<int>(input_shape[2])};
    }
    std::cout << "[" << tag << "] Model input size: " << model_input_size.width << "x" << model_input_size.height
              << std::endl;

    element_type = input_info.GetElementType();
    if (element_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        std::cout << "[" << tag << "] element type: float" << std::endl;
        data_buffer = new float[model_input_size.area() * 3];
    } else if (element_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT16) {
        std::cout << "[" << tag << "] element type: float16" << std::endl;
        data_buffer = new Ort::Float16_t[model_input_size.area() * 3];
    } else {
        throw std::runtime_error(tag + " ONNX input element type not supported");
    }

    Ort::AllocatorWithDefaultOptions allocator;
    input_name_storage.clear();
    input_node_names.clear();
    for (size_t i = 0; i < session.GetInputCount(); ++i) {
        Ort::AllocatedStringPtr name = session.GetInputNameAllocated(i, allocator);
        input_name_storage.emplace_back(name.get());
    }
    for (const auto &s : input_name_storage) {
        input_node_names.push_back(s.c_str());
    }

    output_name_storage.clear();
    output_node_names.clear();
    for (size_t i = 0; i < session.GetOutputCount(); ++i) {
        Ort::AllocatedStringPtr name = session.GetOutputNameAllocated(i, allocator);
        output_name_storage.emplace_back(name.get());
    }
    for (const auto &s : output_name_storage) {
        output_node_names.push_back(s.c_str());
    }
}

inline void FreeBuffer(void *&data_buffer, ONNXTensorElementDataType element_type) {
    if (!data_buffer) {
        return;
    }
    if (element_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
        delete[] reinterpret_cast<float *>(data_buffer);
    } else {
        delete[] reinterpret_cast<Ort::Float16_t *>(data_buffer);
    }
    data_buffer = nullptr;
}

inline void ValidateModelFileExists(const std::string &model_path, const std::string &tag) {
    if (model_path.empty() || !std::filesystem::is_regular_file(model_path)) {
        throw std::runtime_error("[" + tag + "] ONNX model file not found: " + model_path);
    }
}

} // namespace booster_vision::onnx_detail
