#include <chrono>
#include <cstring>
#include <memory>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "gclistener/RoboCupGameControlData.hpp"


#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"

class GCListener : public rclcpp::Node
{
  public:
    GCListener() : Node("gc_listener")
    {
      // VARIABLES
      constexpr int LISTEN_PORT = 3838;

      sock_= socket(AF_INET, SOCK_DGRAM, 0);
      if (sock_<0)
      {
        throw std::runtime_error("Cant create socket" + std::string(strerror(errno)));
      }

      sockaddr_in localAddr{};
      localAddr.sin_family = AF_INET;
      localAddr.sin_port = htons(LISTEN_PORT);
      localAddr.sin_addr.s_addr = INADDR_ANY;

      if(bind(sock_,(sockaddr*)&localAddr, sizeof(localAddr))<0)
      {
        close(sock_);
        throw std::runtime_error("Cant bind the socket" + std::string(strerror(errno)));
      }

      //PARAMETERS
      robot_number = this->declare_parameter<int>("robot_number", 1);    
      team_number = this->declare_parameter<int>("team_number", 0);    

      // PUBLISHERS
      stop_play_publisher       = this-> create_publisher<std_msgs::msg::Int8>(
          "gc_listener/stopped",1);
      game_phase_publisher      = this-> create_publisher<std_msgs::msg::Int8>(
          "gc_listener/game_phase",1);
      state_publisher           = this-> create_publisher<std_msgs::msg::Int8>(
          "gc_listener/state",1);
      set_play_publisher        = this-> create_publisher<std_msgs::msg::Int8>(
          "gc_listener/set_play",1);
      penalty_publisher         = this-> create_publisher<std_msgs::msg::Int8>(
          "gc_listener/penalty",1);
      kicking_team_publisher     = this-> create_publisher<std_msgs::msg::Int8>(
          "gc_listener/kicking_team",1);
      time_publisher            = this-> create_publisher<std_msgs::msg::Int16>(
          "gc_listener/time",1);
      secondary_time_publisher  = this-> create_publisher<std_msgs::msg::Int16>(
          "gc_listener/secondary_time",1);
      
      //TIMERS
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GCListener::timer_callback, this));
      RCLCPP_INFO(get_logger(), "Game Controller listener started");
    }
    ~GCListener()
    {
      if(sock_>= 0)
        close(sock_);
    }

  private:
   // OBJETC DECLARATIONS
   int sock_;
   int team_number;
   int robot_number;
   int array_pose;
   RoboCupGameControlData data{};
   sockaddr_in sender{};
   socklen_t sender_len = sizeof(sender);
   // TOPICS
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr stop_play_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr game_phase_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr set_play_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr penalty_publisher;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr kicking_team_publisher;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr time_publisher;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr secondary_time_publisher;

   //TIMERS
    rclcpp::TimerBase::SharedPtr timer_;

    //Int8
    std_msgs::msg::Int8 state{};
    //Int16
    std_msgs::msg::Int16 time{};

    void timer_callback()
    {
      ssize_t bytes = recvfrom
      (
        sock_,
        &data,
        sizeof(data),
        MSG_DONTWAIT,
        (sockaddr *)&sender,
        &sender_len);
      
      if(bytes <= 0)
        return;

      if(bytes != sizeof(RoboCupGameControlData))
      {
        RCLCPP_WARN(get_logger(), "Received invalid packet");
        return;
      }

      state.data = data.stopped;
      stop_play_publisher->publish(state);

      state.data = data.gamePhase;
      game_phase_publisher->publish(state);

      state.data = data.state;
      state_publisher->publish(state);

      state.data = data.setPlay;
      set_play_publisher->publish(state);

      state.data = data.kickingTeam;
      kicking_team_publisher->publish(state);

      array_pose = (data.teams[0].teamNumber == team_number) ? 0 : 1;

      state.data = data.teams[array_pose].players[robot_number-1].penalty;
    penalty_publisher->publish(state);

      time.data = data.secsRemaining;
      time_publisher->publish(time);

      time.data = data.secondaryTime;
      secondary_time_publisher->publish(time);
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GCListener>());
  rclcpp::shutdown();
  return 0;
}
