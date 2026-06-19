#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <booster_msgs/msg/rpc_req_msg.hpp>

#include <string>
#include <sstream>
#include <random>
#include <chrono>

using namespace std::chrono_literals;

class PantiltToK1Node : public rclcpp::Node
{
public:
    PantiltToK1Node()
    : Node("pantilt_to_k1")
    {
        subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/hardware/head/goal_pose", 10,
            std::bind(&PantiltToK1Node::callback_pantilt_cmd, this, std::placeholders::_1));

        publisher = this->create_publisher<booster_msgs::msg::RpcReqMsg>("/LocoApiTopicReq", 10);
    }

    void spin()
    {
        rclcpp::WallRate loop_rate(50);  // 50 Hz -> 20 ms period

        while (rclcpp::ok()) {
            rclcpp::spin_some(this->shared_from_this());

            if (new_cmd_) {
                new_cmd_ = false;
                auto msg = build_head_msg(pitch_, yaw_);
                publisher->publish(msg);
                RCLCPP_DEBUG(this->get_logger(), "pitch: %.2f , yaw: %.2f", pitch_, yaw_);
            }

            loop_rate.sleep();
        }
    }

private:
    void callback_pantilt_cmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // data[0] = yaw, data[1] = pitch (as in the Python code)
        if (msg->data.size() >= 2) {
            pitch_ = msg->data[1];
            yaw_ = msg->data[0];
            new_cmd_ = true;
        }
    }

    booster_msgs::msg::RpcReqMsg build_head_msg(double pitch, double yaw)
    {
        booster_msgs::msg::RpcReqMsg msg;
        msg.uuid = generate_uuid();

        std::ostringstream header_stream;
        header_stream << R"({"api_id":2004,"expect_response":true})";
        msg.header = header_stream.str();

        std::ostringstream body_stream;
        body_stream << R"({"pitch":)" << pitch
                     << R"(,"yaw":)" << yaw << "}";
        msg.body = body_stream.str();

        return msg;
    }

    std::string generate_uuid()
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 15);
        static std::uniform_int_distribution<> dis2(8, 11);
        static const char* hex = "0123456789abcdef";

        std::string uuid = "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx";
        for (auto& c : uuid) {
            if (c == 'x') {
                c = hex[dis(gen)];
            } else if (c == 'y') {
                c = hex[dis2(gen)];
            }
        }
        return uuid;
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription;
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;

    double pitch_ = 0.0;
    double yaw_ = 0.0;
    bool new_cmd_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PantiltToK1Node>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}
