#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <booster_msgs/msg/rpc_req_msg.hpp>

#include <string>
#include <sstream>
#include <random>
#include <chrono>

using namespace std::chrono_literals;

class TwistToK1Node : public rclcpp::Node
{
public:
    TwistToK1Node()
    : Node("twist_to_k1")
    {
        subscription = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 2, std::bind(&TwistToK1Node::callback_cmd_vel, this, std::placeholders::_1));

        publisher = this->create_publisher<booster_msgs::msg::RpcReqMsg>("/LocoApiTopicReq", 10);
    }

    void spin()
    {
        rclcpp::WallRate loop_rate(10);  // 5 Hz -> 200 ms period
        int no_new_cmd_counter = 0;

        while (rclcpp::ok()) {
            rclcpp::spin_some(this->shared_from_this());

            if (new_cmd_data_) {
                new_cmd_data_ = false;
                no_new_cmd_counter = 0;
                auto msg = build_move_msg(vx_, vy_, vyaw_);
                publisher->publish(msg);
            } else {
                no_new_cmd_counter++;
                if (no_new_cmd_counter > 5) {
                    no_new_cmd_counter = 0;
                    auto msg = build_move_msg(0.0, 0.0, 0.0);
                    publisher->publish(msg);
                    RCLCPP_DEBUG(this->get_logger(), "No cmd_vel received, stopping robot");
                }
            }

            loop_rate.sleep();
        }
    }

private:
    void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        vyaw_ = msg->angular.z;
        new_cmd_data_ = true;
    }

    booster_msgs::msg::RpcReqMsg build_move_msg(double vx, double vy, double vyaw)
    {
        booster_msgs::msg::RpcReqMsg msg;
        msg.uuid = generate_uuid();

        std::ostringstream header_stream;
        header_stream << R"({"api_id":2001,"expect_response":true})";
        msg.header = header_stream.str();

        std::ostringstream body_stream;
        body_stream << R"({"vx":)" << vx
                     << R"(,"vy":)" << vy
                     << R"(,"vyaw":)" << vyaw << "}";
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

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;

    double vx_ = 0.0;
    double vy_ = 0.0;
    double vyaw_ = 0.0;
    bool new_cmd_data_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistToK1Node>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}
