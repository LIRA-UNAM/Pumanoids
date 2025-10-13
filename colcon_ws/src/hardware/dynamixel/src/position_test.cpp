#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"

#define ID_CM730 200
#define ADDR_CM730_WAKE_UP 24

int main(int argc, char **argv){
    //Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std ::make_shared<rclcpp::Node>("position_test");

    // Declare and get the 'id' parameter
    int id = node->declare_parameter("id", 0);
    if(id == 0){
        std::cout << "missing or invalid servo id" << std::endl;
        return -1;
    }

    //Default baudrate and port 
    int baudrate = node->declare_parameter("baudrate",1000000);
    std::string port = node->declare_parameter<std::string>("port","/dev/ttyUSB0");

    //Set port, select protocol and set baudrate
    dynamixel::PortHandler      *portHandler    = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler    *packetHandler  = dynamixel::PacketHandler::getPacketHandler(1.0);
    portHandler->setBaudRate(baudrate);

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint16_t info;
    int present_position_addr=36;

    /*---------------------------------------
    |      WAKE UP CM730 (ADDR 24)          |
    -----------------------------------------*/
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 
                                                    ID_CM730, 
                                                    ADDR_CM730_WAKE_UP , 
                                                    1, 
                                                    &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
        std::cout << "CM730.->Commnunication problem while turning on dynamixel power." << std::endl;
    if(dxl_error != 0){
        std::cout << "CM730.->Status error after turning on dynamixel power: " << int(dxl_error) << std::endl;
        return -1;
    }
    std::cout << "\t Servo ID: " << id << std::endl;

    while(rclcpp::ok()){
        /*---------------------------------------
        |      READ ID POSITION (ADDR 36)        |
        -----------------------------------------*/
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, 
                                                       id,
                                                       present_position_addr,
                                                       &info,
                                                       &dxl_error);
        if(dxl_comm_result != COMM_SUCCESS){
            std::cout << "Communication Error" << std::endl;
            return -1;
        }
        std::cout << "\r\t Position: " << int(info) << "  " << std::flush;

    }
    std::cout << std::endl;
    portHandler->closePort();
    rclcpp::shutdown();
    return 0;
}