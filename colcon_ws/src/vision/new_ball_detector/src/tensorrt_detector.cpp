#include "new_ball_detector/tensorrt_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <numeric>

class Logger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char *msg) noexcept override {
    if (severity <= Severity::kWARNING)
      std::cout << msg << std::endl;
  }
};

class YoloDetector::Impl {
public:
  Impl(const std::string &engine_path, const std::vector<std::string> &class_names)
      : class_names_(class_names) {
    try {
      cudaFree(0);
      loadEngine(engine_path);
      prepareBuffers();
    } catch (const std::exception &e){
        std::cerr << "Failed to initialize detector: " << e.what() << '\n';
        throw;
    }
  }
  
  std::pair<int, int> getInputDims() const { return {input_w_, input_h_}; }

  ~Impl() {
    cudaFreeHost(input_cpu_);
    cudaFreeHost(output_cpu_);
    cudaFree(input_gpu_);
    cudaFree(output_gpu_);
    cudaStreamDestroy(stream_);
  }

  std::pair<int, int> getInputSize() const { return {input_w_, input_h_}; }

  std::vector<Detection> run(const cv::Mat &bgr) {
    // Preprocess (Your existing pre-processing is correct)
    // blobFromImage(image, scaleFactor, size, mean, swapRB, crop)
    cv::Mat blob = cv::dnn::blobFromImage(bgr, 1.0 / 255.0, cv::Size(input_w_, input_h_), cv::Scalar(), true, false);

    // The resulting 'blob' is already contiguous in memory and in NCHW format!
    memcpy(input_cpu_, blob.ptr<float>(), input_size_);
    
    // Upload input
    cudaMemcpyAsync(input_gpu_, input_cpu_, input_size_,
                    cudaMemcpyHostToDevice, stream_);

    // --- TensorRT 8+ inference ---
    context_->setInputTensorAddress(input_name_.c_str(), input_gpu_);
    context_->setOutputTensorAddress(output_name_.c_str(), output_gpu_);
    context_->enqueueV3(stream_);
    // ------------------------------

    // Download output
    cudaMemcpyAsync(output_cpu_, output_gpu_, output_size_,
                    cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);


    nvinfer1::Dims actualOutDims = context_->getTensorShape(output_name_.c_str());
    
    // YOLOv8 ONNX shape: [1, 4 + num_classes, num_anchors]
    int numChannels   = actualOutDims.d[1]; 
    int numDetections = actualOutDims.d[2]; 
    
    // YOLOv8 has NO objectness score, just 4 bbox coords + N class scores
    const int numClasses = numChannels - 4; 

    float *output = reinterpret_cast<float *>(output_cpu_);

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    // YOLOv8 tensor memory layout is contiguous by channel, then anchor:
    // output[channel * numDetections + anchor]
    for (int i = 0; i < numDetections; ++i) {
      float max_class_conf = 0.0f;
      int class_id = -1;

      // Classes start at channel 4
      for (int c = 0; c < numClasses; ++c) {
        float conf = output[(4 + c) * numDetections + i];
        if (conf > max_class_conf) {
            max_class_conf = conf;
            class_id = c;
        }
      }

      // Pre-filter threshold to speed up NMS processing
      if (max_class_conf > 0.40f) { 
        float cx = output[0 * numDetections + i];
        float cy = output[1 * numDetections + i];
        float w  = output[2 * numDetections + i];
        float h  = output[3 * numDetections + i];

        // Convert center format to top-left format for OpenCV's NMS
        int left = static_cast<int>(cx - w / 2.0f);
        int top  = static_cast<int>(cy - h / 2.0f);
        int width = static_cast<int>(w);
        int height = static_cast<int>(h);

        classIds.push_back(class_id);
        confidences.push_back(max_class_conf);
        boxes.push_back(cv::Rect(left, top, width, height));
      }
    }

    // Apply Non-Maximum Suppression (NMS) to eliminate overlapping boxes
    std::vector<int> indices;
    float score_threshold = 0.40f;
    float nms_threshold = 0.45f;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold, nms_threshold, indices);

    std::vector<Detection> detections;
    detections.reserve(indices.size());

    for (int idx : indices) {
      Detection d;
      d.class_name = class_names_[classIds[idx]];
      d.confidence = confidences[idx];
      
      // Re-calculate center points
      float w = static_cast<float>(boxes[idx].width);
      float h = static_cast<float>(boxes[idx].height);
      float cx = static_cast<float>(boxes[idx].x) + w / 2.0f;
      float cy = static_cast<float>(boxes[idx].y) + h / 2.0f;
      
      // Normalize coordinates to [0.0, 1.0] range
      // so ball_detector.cpp can correctly multiply them by the camera feed's width/height
      d.center_x = cx / static_cast<float>(input_w_);
      d.center_y = cy / static_cast<float>(input_h_);
      d.width    = w / static_cast<float>(input_w_);
      d.height   = h / static_cast<float>(input_h_);
      
      detections.push_back(d);
    }

    return detections;
  }

private:
  void loadEngine(const std::string &path) {
    std::cout << "TensorRT runtime version: " << getInferLibVersion() << std::endl;
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open engine file: " + path);
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> engine_data(size);
    file.read(engine_data.data(), size);
    std::cout << "Engine file loaded: " << size << " bytes" << '\n';

    Logger logger;

    std::cout << "Creating InferRuntime..." << '\n';
    runtime_ = nvinfer1::createInferRuntime(logger);
    std::cout << "InferRuntime created" << '\n';

    std::cout << "Deserializing engine..." << '\n';
    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), size);
    std::cout << "Engine deserialized" << '\n';

    std::cout << "Creating execution context..." << '\n';
    context_ = engine_->createExecutionContext();
    if (!context_) throw std::runtime_error("Failed to create execution context");
    std::cout << "Context created" << '\n';
  }

  void prepareBuffers() {
    input_name_  = engine_->getIOTensorName(0);
    output_name_ = engine_->getIOTensorName(1);

    std::cout << "Input name: " << input_name_ << ", Output name: " << output_name_ << std::endl;

    // For static engines, getTensorShape returns the exact dimensions.
    // For dynamic engines, it returns the maximum shape after setting an optimization profile.
    nvinfer1::Dims inputDims = engine_->getTensorShape(input_name_.c_str());
    nvinfer1::Dims outputDims = engine_->getTensorShape(output_name_.c_str());

    std::cout << "Input dims: ";
    for (int i = 0; i < inputDims.nbDims; ++i) std::cout << inputDims.d[i] << " ";
    std::cout << std::endl;
    std::cout << "Output dims: ";
    for (int i = 0; i < outputDims.nbDims; ++i) std::cout << outputDims.d[i] << " ";
    std::cout << std::endl;

    // Assuming NCHW: [batch, channels, height, width]
    input_w_ = inputDims.d[3];
    input_h_ = inputDims.d[2];
    input_size_ = 1 * 3 * input_w_ * input_h_ * sizeof(float);

    int maxElements = 1;
    for (int i = 0; i < outputDims.nbDims; ++i) {
        maxElements *= outputDims.d[i];
    }
    output_size_ = maxElements * sizeof(float);

    cudaError_t err;

    std::cout << "Allocating input GPU buffer (" << input_size_ << " bytes)..." << '\n';
    err = cudaMalloc(&input_gpu_, input_size_);
    if (err != cudaSuccess) throw std::runtime_error("cudaMalloc input failed");
    std::cout << "  returned " << cudaGetErrorString(err) << '\n';

    std::cout << "Allocating output GPU buffer (" << output_size_ << " bytes)..." << '\n';
    err = cudaMalloc(&output_gpu_, output_size_);
    if (err != cudaSuccess) throw std::runtime_error("cudaMalloc output failed");
    std::cout << "  returned " << cudaGetErrorString(err) << '\n';

    std::cout << "Allocating input GPU host buffer (" << input_size_ << " bytes)..." << '\n';
    err = cudaMallocHost(&input_cpu_, input_size_);
    if (err != cudaSuccess) throw std::runtime_error("cudaMallocHost input failed");
    std::cout << "  returned " << cudaGetErrorString(err) << '\n';

    std::cout << "Allocating output GPU host buffer (" << output_size_ << " bytes)..." << '\n';
    err = cudaMallocHost(&output_cpu_, output_size_);
    if (err != cudaSuccess) throw std::runtime_error("cudaMallocHost output failed");
    std::cout << "  returned " << cudaGetErrorString(err) << '\n';

    std::cout << "Creating CUDA stream..." << '\n';
    cudaStreamCreate(&stream_);
    std::cout << "Stream created." << '\n';
  }

  nvinfer1::IRuntime* runtime_;
  nvinfer1::ICudaEngine* engine_;
  nvinfer1::IExecutionContext* context_;
  cudaStream_t stream_;
  void* input_gpu_;
  void* output_gpu_;
  float* input_cpu_;
  float* output_cpu_;
  size_t input_size_;
  size_t output_size_;
  int input_w_, input_h_;
  std::vector<void*> buffers_;
  std::vector<std::string> class_names_;
  std::string input_name_;
  std::string output_name_;
};

YoloDetector::YoloDetector(const std::string &engine_path,
                           const std::vector<std::string> &class_names)
    : pimpl_(new Impl(engine_path, class_names)) {}

YoloDetector::~YoloDetector() = default;

std::vector<Detection> YoloDetector::detect(const cv::Mat &bgr_image) {
  return pimpl_->run(bgr_image);
}

std::pair<int, int> YoloDetector::getInputSize() const {
  return pimpl_->getInputDims();
}
