#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <zlib.h>  // For Zlib compression

class CompressedPointCloudPublisher : public rclcpp::Node
{
public:
    CompressedPointCloudPublisher() : Node("compressed_point_cloud_publisher")
    {
        // Publisher for compressed data
        compressed_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/depth/points/compressed", rclcpp::SensorDataQoS());

        // Subscribe to the raw PointCloud2 topic
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", rclcpp::SensorDataQoS(),
            std::bind(&CompressedPointCloudPublisher::cloudCallback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Compressed PointCloud Publisher Node has been started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_point_cloud_pub_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Compress the incoming point cloud data using zlib
        std::vector<uint8_t> compressed_data;

        if (compressPointCloud(msg, compressed_data)) {
            // Create a CompressedImage message to hold the compressed data
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.format = "png";  // Specify format (e.g., png, jpeg)
            compressed_msg.data = compressed_data;

            // Publish the compressed point cloud
            compressed_point_cloud_pub_->publish(compressed_msg);
            RCLCPP_INFO(this->get_logger(), "Published compressed point cloud.");
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compress PointCloud2.");
        }
    }

    bool compressPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, std::vector<uint8_t>& compressed_data)
    {
        // Step 1: Serialize the PointCloud2 data into a raw byte buffer
        const uint8_t* raw_data = msg->data.data();

        // Step 2: Compress the raw data using zlib
        uLongf compressed_size = compressBound(msg->data.size());
        compressed_data.resize(compressed_size);

        int result = compress(compressed_data.data(), &compressed_size, raw_data, msg->data.size());

        if (result == Z_OK) {
            compressed_data.resize(compressed_size); // Adjust size to actual compressed size
            return true;
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Zlib compression failed with error code %d", result);
            return false;
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompressedPointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}

