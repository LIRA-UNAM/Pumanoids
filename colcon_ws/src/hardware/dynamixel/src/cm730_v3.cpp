#include "rclcpp/rclcpp.hpp"
#include "cm730_utils.hpp"


int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    
    CM730::CM730Node n("/dev/SERVO_COMM");

    if(!node.start()) return -1;

    return 0;
}
