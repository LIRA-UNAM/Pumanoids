#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    std::cout << "INITIALIZING JUSTINA GUI NODE ..." << std::endl;
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<RclComm>());
    // rclcpp::shutdown();
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}


// #include "rclcpp/rclcpp.hpp"
// #include <chrono>
// #include <QThread>

// class MinimalTimerNode : public QThread, rclcpp::Node
// {
// public:
//     MinimalTimerNode() : Node("minimal_timer_node")
//     {
//         using namespace std::chrono_literals;
//         timer_ = this->create_wall_timer(
//             1s, // 1-second period
//             std::bind(&MinimalTimerNode::timer_callback, this)
//         );
//         RCLCPP_INFO(this->get_logger(), "MinimalTimerNode created with a 1-second wall timer.");
//     }

// private:
//     void timer_callback()
//     {
//         RCLCPP_INFO(this->get_logger(), "Timer callback: Hello from the timer!");
//     }

//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MinimalTimerNode>());
//     rclcpp::shutdown();
//     return 0;
// }
