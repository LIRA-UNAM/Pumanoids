#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

#include "brain.h"

#define HZ 100

using namespace std;

int main(int argc, char **argv)
{

    // Initialize ROS 2.
    rclcpp::init(argc, argv);

    // Create the Brain node.
    std::shared_ptr<Brain> brain = std::make_shared<Brain>();

    // Load parameters, construct the behavior tree, and initialize subsystems.
    brain->init();

    // Run brain.tick in a dedicated thread.
    thread t([&brain]() {
        while (rclcpp::ok()) {
            auto start_time = brain->get_clock()->now();
            brain->tick();
            auto end_time = brain->get_clock()->now();
            auto duration = (end_time - start_time).nanoseconds() / 1000000.0; // Convert to milliseconds.
            if (brain->log->shouldLog(
                    "brain_tick_timeseries", brain->config->rerunLogTimeseriesHz)) {
                brain->log->setTimeNow();
                brain->log->log("performance/brain_tick", rerun::Scalar(duration));
            }
            this_thread::sleep_for(chrono::milliseconds(static_cast<int>(1000 / HZ)));
        }
    });

    // Process joystick and GameController callbacks in a separate thread.
    thread t1([&brain, &argc, &argv]() {
        // Use an independent context.
        auto context = rclcpp::Context::make_shared();
        context->init(argc, argv);
        rclcpp::NodeOptions opt;
        opt.context(context);
        auto node = rclcpp::Node::make_shared("brain_node_ext", opt);
        auto sub1 = node->create_subscription<booster_interface::msg::RemoteControllerState>("/remote_controller_state", 10, bind(&Brain::joystickCallback, brain, std::placeholders::_1));
        auto sub2 = node->create_subscription<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 10, bind(&Brain::gameControlCallback, brain, std::placeholders::_1));

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    });

    // Use a single-threaded executor.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(brain);
    executor.spin();

    t.join();
    t1.join();
    rclcpp::shutdown();
    return 0;
}
