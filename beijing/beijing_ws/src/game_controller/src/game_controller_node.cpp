#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "game_controller_node.h"

GameControllerNode::GameControllerNode(string name) : rclcpp::Node(name)
{
    _socket = -1;

    declare_parameter<int>("port", GAMECONTROLLER_DATA_PORT);
    declare_parameter<bool>("enable_ip_white_list", false);
    declare_parameter<vector<string>>("ip_white_list", vector<string>{});

    get_parameter("port", _port);
    RCLCPP_INFO(get_logger(), "[get_parameter] port: %d", _port);
    get_parameter("enable_ip_white_list", _enable_ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] enable_ip_white_list: %d", _enable_ip_white_list);
    get_parameter("ip_white_list", _ip_white_list);
    RCLCPP_INFO(get_logger(), "[get_parameter] ip_white_list(len=%zu)", _ip_white_list.size());
    for (size_t i = 0; i < _ip_white_list.size(); ++i)
    {
        RCLCPP_INFO(get_logger(), "[get_parameter]     --[%zu]: %s", i, _ip_white_list[i].c_str());
    }

    _publisher = create_publisher<game_controller_interface::msg::GameControlData>(
        "/robocup/game_controller", 10);
}

GameControllerNode::~GameControllerNode()
{
    _running = false;
    if (_thread.joinable())
    {
        _thread.join();
    }

    if (_socket >= 0)
    {
        close(_socket);
        _socket = -1;
    }
}

void GameControllerNode::init()
{
    _socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_socket < 0)
    {
        RCLCPP_ERROR(get_logger(), "socket failed: %s", strerror(errno));
        throw runtime_error(strerror(errno));
    }

    const timeval receive_timeout{1, 0};
    if (setsockopt(
            _socket, SOL_SOCKET, SO_RCVTIMEO,
            &receive_timeout, sizeof(receive_timeout)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "setting socket receive timeout failed: %s", strerror(errno));
        throw runtime_error(strerror(errno));
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(_port);

    if (bind(_socket, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "bind failed: %s (port=%d)", strerror(errno), _port);
        throw runtime_error(strerror(errno));
    }

    RCLCPP_INFO(
        get_logger(), "Listening for GameController v%d UDP packets on 0.0.0.0:%d (size=%zu)",
        GAMECONTROLLER_STRUCT_VERSION, _port, sizeof(RoboCupGameControlData));
    _running = true;
    _thread = thread(&GameControllerNode::spin, this);
}

void GameControllerNode::spin()
{
    RoboCupGameControlData data{};
    uint8_t buffer[sizeof(RoboCupGameControlData) + 1]{};
    game_controller_interface::msg::GameControlData msg;
    uint64_t datagrams_received = 0;
    uint64_t packets_accepted = 0;
    auto status_time = chrono::steady_clock::now();

    while (rclcpp::ok() && _running)
    {
        sockaddr_in remote_addr{};
        socklen_t remote_addr_len = sizeof(remote_addr);
        const ssize_t received = recvfrom(
            _socket, buffer, sizeof(buffer), 0,
            reinterpret_cast<sockaddr *>(&remote_addr), &remote_addr_len);
        if (received < 0)
        {
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK && rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "receiving UDP message failed: %s", strerror(errno));
            }
            const auto now = chrono::steady_clock::now();
            if ((errno == EAGAIN || errno == EWOULDBLOCK) &&
                now - status_time >= chrono::seconds(5))
            {
                RCLCPP_INFO(
                    get_logger(),
                    "GameController status: no datagram received in the last 5 s "
                    "(total=%llu accepted=%llu)",
                    static_cast<unsigned long long>(datagrams_received),
                    static_cast<unsigned long long>(packets_accepted));
                status_time = now;
            }
            continue;
        }
        ++datagrams_received;

        const string remote_ip = inet_ntoa(remote_addr.sin_addr);
        if (received != static_cast<ssize_t>(sizeof(data)))
        {
            RCLCPP_WARN(
                get_logger(), "packet from %s has invalid length=%zd, expected=%zu",
                remote_ip.c_str(), received, sizeof(data));
            continue;
        }
        memcpy(&data, buffer, sizeof(data));
        if (memcmp(data.header, GAMECONTROLLER_STRUCT_HEADER, sizeof(data.header)) != 0)
        {
            RCLCPP_WARN(get_logger(), "packet from %s has invalid header", remote_ip.c_str());
            continue;
        }
        if (data.version != GAMECONTROLLER_STRUCT_VERSION)
        {
            RCLCPP_WARN(
                get_logger(), "packet from %s has invalid version=%u, expected=%d",
                remote_ip.c_str(), static_cast<unsigned>(data.version),
                GAMECONTROLLER_STRUCT_VERSION);
            continue;
        }
        const bool distinctTeams =
            data.teams[0].teamNumber != data.teams[1].teamNumber;
        const bool validKickingTeam =
            data.kickingTeam == KICKING_TEAM_NONE ||
            data.kickingTeam == data.teams[0].teamNumber ||
            data.kickingTeam == data.teams[1].teamNumber;
        if (data.playersPerTeam == 0 || data.playersPerTeam > MAX_NUM_PLAYERS ||
            data.competitionType > COMPETITION_TYPE_LARGE || data.stopped > 1 ||
            data.gamePhase > GAME_PHASE_TIMEOUT || data.state > STATE_FINISHED ||
            data.setPlay > SET_PLAY_CORNER_KICK || data.firstHalf > 1 ||
            !distinctTeams || !validKickingTeam)
        {
            RCLCPP_WARN(get_logger(), "packet from %s contains invalid field values", remote_ip.c_str());
            continue;
        }
        if (!check_ip_white_list(remote_ip))
        {
            RCLCPP_WARN(
                get_logger(), "packet from %s is not in the GameController IP allowlist",
                remote_ip.c_str());
            continue;
        }

        handle_packet(data, msg);
        _publisher->publish(msg);
        ++packets_accepted;
        RCLCPP_DEBUG(
            get_logger(), "handled packet from %s, packet_number=%u",
            remote_ip.c_str(), static_cast<unsigned>(data.packetNumber));

        // INFO is intentionally kept quiet for every packet.  A periodic
        // status line makes a redirected game_controller.log observable while
        // retaining the normal low-noise behavior.
        const auto now = chrono::steady_clock::now();
        if (now - status_time >= chrono::seconds(5))
        {
            RCLCPP_INFO(
                get_logger(),
                "GameController status: datagrams=%llu accepted=%llu last_source=%s",
                static_cast<unsigned long long>(datagrams_received),
                static_cast<unsigned long long>(packets_accepted),
                remote_ip.c_str());
            status_time = now;
        }
    }
}

bool GameControllerNode::check_ip_white_list(const string &ip) const
{
    if (!_enable_ip_white_list)
    {
        return true;
    }
    for (const auto &allowed_ip : _ip_white_list)
    {
        if (ip == allowed_ip)
        {
            return true;
        }
    }
    return false;
}

void GameControllerNode::handle_packet(
    const RoboCupGameControlData &data,
    game_controller_interface::msg::GameControlData &msg) const
{
    for (size_t i = 0; i < sizeof(data.header); ++i)
    {
        msg.header[i] = data.header[i];
    }
    msg.version = data.version;
    msg.packet_number = data.packetNumber;
    msg.players_per_team = data.playersPerTeam;
    msg.competition_type = data.competitionType;
    msg.stopped = data.stopped != 0;
    msg.game_phase = data.gamePhase;
    msg.state = data.state;
    msg.set_play = data.setPlay;
    msg.first_half = data.firstHalf != 0;
    msg.kicking_team = data.kickingTeam;
    msg.secs_remaining = data.secsRemaining;
    msg.secondary_time = data.secondaryTime;

    for (size_t i = 0; i < 2; ++i)
    {
        msg.teams[i].team_number = data.teams[i].teamNumber;
        msg.teams[i].field_player_colour = data.teams[i].fieldPlayerColour;
        msg.teams[i].goalkeeper_colour = data.teams[i].goalkeeperColour;
        msg.teams[i].goalkeeper = data.teams[i].goalkeeper;
        msg.teams[i].score = data.teams[i].score;
        msg.teams[i].penalty_shot = data.teams[i].penaltyShot;
        msg.teams[i].single_shots = data.teams[i].singleShots;
        msg.teams[i].message_budget = data.teams[i].messageBudget;

        for (size_t j = 0; j < MAX_NUM_PLAYERS; ++j)
        {
            msg.teams[i].players[j].penalty = data.teams[i].players[j].penalty;
            msg.teams[i].players[j].secs_till_unpenalised =
                data.teams[i].players[j].secsTillUnpenalised;
            msg.teams[i].players[j].cautions = data.teams[i].players[j].cautions;
        }
    }
}
