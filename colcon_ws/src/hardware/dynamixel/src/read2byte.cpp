#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    //Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("read2byte");

    //Declare variables
    int id;
    int addr;

    // Declare parameters first
    node->declare_parameter("id", 0);
    node->declare_parameter("addr", 0);

    if(!node->has_parameter("id")){
        std::cout<<"missing servo id"<<std::endl;
        return -1;
    }
    if(!node->has_parameter("addr")){
        std::cout<<"missing address"<<std::endl;
        return -1;
    }
    if(!node->get_parameter("id",id )){
        std::cout<<"Invalid servo id"<<std::endl;
        return -1;
    }
    if(!node->get_parameter("addr",addr)){
        std::cout<<"Invalid address"<<std::endl;
        return -1;
    }

    //Default baudrate and port
    int baudrate = node->declare_parameter("baudrate",1000000);
    std::string port = node->declare_parameter<std::string>("port","/dev/ttyUSB0");

    //Set port, select protocol and set baudrate
    dynamixel::PortHandler   *portHandler   = dynamixel::PortHandler::getPortHandler(port.c_str());
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);
    portHandler->setBaudRate(baudrate);

    uint8_t  dxl_error_read      = 0;
    int      dxl_comm_result = COMM_TX_FAIL;
    uint16_t info;

    //Read value of 2 bytes
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id,addr,&info, &dxl_error_read);

    if(dxl_comm_result != COMM_SUCCESS)
    {
        std::cout<<"Comunication error"<<std::endl;
        return -1;
    }

    if(dxl_error_read & 0x01)
    {
        std::cout<<"Input voltage error"<<std::endl;
    }
    if(dxl_error_read & 0x02)
    {
        std::cout<<"Angle limit error"<<std::endl;
    }
    if(dxl_error_read & 0x04)
    {
        std::cout<<"Overheating error"<<std::endl;
    }
    if(dxl_error_read & 0x08)
    {
        std::cout<<"Range error"<<std::endl;
    }
    if(dxl_error_read & 0x10)
    {
        std::cout<<"CheckSum error"<<std::endl;
    }
    if(dxl_error_read & 0x20)
    {
        std::cout<<"Overload error"<<std::endl;
    }
    if(dxl_error_read & 0x40)
    {
        std::cout<<"Instruction error"<<std::endl;
    }

    std::cout<<"Data: "<<int(info)<<"\tid: "<<id<<"\taddress: "<<addr<<"\tBaudRate: "<<baudrate<<"\tPort: "<<port
    <<"\tError code: "<<int(dxl_error_read)<<std::endl;
    
    portHandler->closePort();
    rclcpp::shutdown();
    return 0;
}
    