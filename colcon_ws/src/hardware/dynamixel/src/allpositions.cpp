#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include<iostream>
#include<string>

int main(int argc, char **argv){
    //Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std ::make_shared<rclcpp::Node>("allpositions");
    //Default baudrate and port
    int baudrate = node->declare_parameter("baudrate",1000000);
    std::string port = node->declare_parameter<std::string>("port","/dev/ttyUSB0");

    //Set port, select protocol and set baudrate
    dynamixel::PortHandler      *portHandler    = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler    *packetHandler  = dynamixel::PacketHandler::getPacketHandler(1.0);
    portHandler->setBaudRate(baudrate);

    uint8_t dxl_error_read = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint16_t info;
    int present_position_addr=36;

    for (int id =1; id<=20; id++)
    {
         dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, present_position_addr, &info, &dxl_error_read);
    if(dxl_comm_result != COMM_SUCCESS)
    {
        std::cout << "Communication Error" << std::endl;
        return -1;
    }
    std::cout<<"id:"<<id<<"    "<<"Position:"<<int (info)<< std::endl;
    }
    std::cout << std::endl;
    portHandler->closePort();
    return 0;
}
