
/*
 * nv12_converter_node.cpp
 * Package: boosterk1_image_proc
 * Node: nv12_converter_node
 * This node converts the raw image encoded in nv12 and publishes it
 * as rgb8 through the /camera/color/image_raw topic.
 * Written by Sebastian Garcia
 * Developed at LIRA UNAM
 * https://lira.unam.mx/
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <jetson-utils/cudaColorspace.h>
#include <jetson-utils/cudaMappedMemory.h>

class NV12ConverterNode : public rclcpp::Node {
public:
    NV12ConverterNode() : Node("nv12_converter_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/booster_camera_bridge/StereoNetNode/rectified_image", 10, 
            std::bind(&NV12ConverterNode::image_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 10);
        RCLCPP_INFO(this->get_logger(), "GPU Converter Started with Memory Mapping...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        size_t nv12_size = msg->width * msg->height * 1.5;
        size_t rgb_size = msg->width * msg->height * sizeof(uchar3);

        if (!cuda_in_ || last_width_ != msg->width || last_height_ != msg->height) {
            if (cuda_in_) cudaFreeHost(cuda_in_);
            if (cuda_intermediate_) cudaFreeHost(cuda_intermediate_);
            if (cuda_out_) cudaFreeHost(cuda_out_);

            cudaAllocMapped(&cuda_in_, nv12_size);
            cudaAllocMapped(&cuda_intermediate_, rgb_size);
            cudaAllocMapped(&cuda_out_, rgb_size);

            last_width_ = msg->width;
            last_height_ = msg->height;
        }

        memcpy(cuda_in_, msg->data.data(), nv12_size);

        if (CUDA_FAILED(cudaConvertColor(cuda_in_, IMAGE_NV12,
                                         cuda_intermediate_, IMAGE_RGB8,
                                         msg->width, msg->height))) {
            RCLCPP_ERROR(this->get_logger(), "GPU Conversion Failed");
            return;
        }
        if (CUDA_FAILED(cudaConvertColor(cuda_intermediate_, IMAGE_RGB8,
                                     cuda_out_, IMAGE_BGR8,
                                     msg->width, msg->height))) {
            RCLCPP_ERROR(this->get_logger(), "Step 2: RGB to BGR Failed");
            return;
        }

        auto out_msg = std::make_shared<sensor_msgs::msg::Image>();
        out_msg->header = msg->header;
        out_msg->height = msg->height;
        out_msg->width = msg->width;
        out_msg->encoding = "bgr8";
        out_msg->step = msg->width * 3;
        out_msg->data.resize(rgb_size);
        memcpy(out_msg->data.data(), cuda_out_, rgb_size);

        pub_->publish(*out_msg);
    }

    void* cuda_in_ = nullptr;
    void* cuda_intermediate_ = nullptr;
    void* cuda_out_ = nullptr;
    uint32_t last_width_ = 0;
    uint32_t last_height_ = 0;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NV12ConverterNode>());
    rclcpp::shutdown();
    return 0;
}
