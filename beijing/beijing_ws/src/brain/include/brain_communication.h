#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdexcept>


#include "RoboCupGameControlData.h"
#include "team_communication_msg.h"
#include "utils/print.h"


class Brain; // Forward declaration

using namespace std;


class BrainCommunication
{
public:
    BrainCommunication(Brain *argBrain);
    ~BrainCommunication();
    
    void initCommunication();
    uint64_t bootId() const { return _boot_id; }

private:
    Brain *brain;

    void initGameControllerUnicast();
    std::thread _gamecontrol_unicast_thread;
    void unicastToGameController();
    void clearupGameControllerUnicast();
    std::atomic_bool _unicast_gamecontrol_flag{false};
    int _gc_send_socket = -1;
    sockaddr_in _gcsaddr;
    RoboCupGameControlReturnData gc_return_data;
    static constexpr int BROADCAST_GAME_CONTROL_INTERVAL_MS = 1000;

    const char* MULTICAST_ADDR = "239.255.255.250"; // Multicast address
    int _discovery_msg_id = 0;

    void initDiscoveryBroadcast();
    void clearupDiscoveryBroadcast();
    void broadcastDiscovery();
    std::thread _discovery_broadcast_thread;
    std::atomic_bool _broadcast_discovery_flag{false};
    int _discovery_send_socket = -1;
    int _discovery_udp_port = 0;
    sockaddr_in _saddr;
    static constexpr int BROADCAST_DISCOVERY_INTERVAL_MS = 1000;


    void initDiscoveryReceiver();
    void clearupDiscoveryReceiver();
    void spinDiscoveryReceiver();
    std::atomic_bool _receive_discovery_flag{false};
    std::thread _discovery_recv_thread;
    int _discovery_recv_socket = -1;


    struct TeammateInfo {
        uint32_t ip;
        int playerId;
        rclcpp::Time lastUpdate;
    };
    std::map<uint32_t, TeammateInfo> _teammate_addresses; // IPv4 address -> teammate
    std::mutex _teammate_addresses_mutex;
    static constexpr int TEAMMATE_TIMEOUT_MS = 20 * 1000; // 20-second timeout
    
    void cleanupExpiredTeammates();

    void initCommunicationUnicast();
    void clearupCommunicationUnicast();
    void unicastCommunication();
    uint32_t _team_communication_sequence = 0;
    uint64_t _boot_id = 0;
    std::atomic_bool _unicast_communication_flag{false};
    std::thread _unicast_thread;
    int _unicast_socket = -1;
    int _unicast_udp_port = 0;
    sockaddr_in _unicast_saddr;
    int teamCommunicationIntervalMs() const;
    static constexpr double DEFAULT_TEAM_COMMUNICATION_RATE_HZ = 20.0;
    static constexpr double MAX_TEAM_COMMUNICATION_RATE_HZ = 20.0;


    void initCommunicationReceiver();
    void clearupCommunicationReceiver();
    void spinCommunicationReceiver();
    std::atomic_bool _receive_communication_flag{false};
    std::thread _communication_recv_thread;
    int _communication_recv_socket = -1;
    int _communication_recv_port = 0;
};
