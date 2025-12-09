#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudDownsampler : public rclcpp::Node
{
public:
    PointCloudDownsampler() : Node("point_cloud_downsampler")
    {
        // Create a subscriber for the raw point cloud
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", rclcpp::SensorDataQoS(),
            std::bind(&PointCloudDownsampler::cloudCallback, this, std::placeholders::_1)
        );
        
        // Publisher for the downsampled point cloud
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points/downsampled", rclcpp::SensorDataQoS());
        
        RCLCPP_INFO(this->get_logger(), "Point Cloud Downsampler Node started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud!");
            return;
        }

        // Log number of points in the original point cloud
        RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points.", cloud->size());

        // Apply voxel grid filter for downsampling
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);  // Set the voxel size (adjust as needed)
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        voxel_grid.filter(*cloud_filtered);

        // Log number of points in the downsampled point cloud
        RCLCPP_INFO(this->get_logger(), "Downsampled point cloud has %zu points.", cloud_filtered->size());

        // Convert the downsampled PCL cloud back to ROS PointCloud2
        sensor_msgs::msg::PointCloud2 cloud_filtered_ros;
        pcl::toROSMsg(*cloud_filtered, cloud_filtered_ros);

        // Maintain the frame ID
        cloud_filtered_ros.header.frame_id = msg->header.frame_id;

        // Publish the downsampled cloud
        point_cloud_pub_->publish(cloud_filtered_ros);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudDownsampler>());
    rclcpp::shutdown();
    return 0;
}

