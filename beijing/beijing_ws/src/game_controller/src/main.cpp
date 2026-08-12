#include "game_controller_node.h"

using namespace std;


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create and initialize the node, then spin.
    auto node = make_shared<GameControllerNode>("game_controller_node");

    // Enter the executor after initialization.
    node->init();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
