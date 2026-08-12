#include "brain.h"
#include "brain_communication.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace {

constexpr uint8_t BOOT_CONFIRMATION_PACKET_COUNT = 3;
// At the supported minimum rate (0.1 Hz), three packets span 20 seconds.
constexpr auto BOOT_CONFIRMATION_WINDOW = std::chrono::seconds(30);

bool sequenceIsNewer(uint32_t candidate, uint32_t reference)
{
    const uint32_t distance = candidate - reference;
    return distance != 0 && distance < 0x80000000U;
}

} // namespace

BrainCommunication::BrainCommunication(Brain *argBrain) : brain(argBrain)
{
    const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
    _boot_id = static_cast<uint64_t>(ticks) ^
        (static_cast<uint64_t>(reinterpret_cast<uintptr_t>(this)) << 1);
    if (_boot_id == 0) _boot_id = 1;
}

BrainCommunication::~BrainCommunication()
{
    clearupGameControllerUnicast();
    clearupDiscoveryBroadcast();
    clearupDiscoveryReceiver();
    clearupCommunicationUnicast();
    clearupCommunicationReceiver();
}


void BrainCommunication::initCommunication()
{
    initGameControllerUnicast();
    if (brain->config->enableCom)
    {
        cout << RED_CODE << "Communication enabled." << RESET_CODE << endl;
        _discovery_udp_port = 20000 + brain->config->teamId;
        _unicast_udp_port = 30000 + brain->config->teamId;

        initDiscoveryBroadcast();
        initDiscoveryReceiver();
        initCommunicationUnicast();
        initCommunicationReceiver();
    }
    else
    {
        cout << RED_CODE << "Communication disabled." << RESET_CODE << endl;
    }
}
    

void BrainCommunication::initGameControllerUnicast()
{
    try
    {
        _gc_send_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_gc_send_socket < 0)
        {
        cout << RED_CODE << format("gc socket failed: %s", strerror(errno))
            << RESET_CODE << endl;
        throw std::runtime_error(strerror(errno));
        }
        // Configure the destination address.
        string gamecontrol_ip = brain->get_parameter("game_control_ip").as_string();
        cout << GREEN_CODE << format("GameControl IP: %s", gamecontrol_ip.c_str())
            << RESET_CODE << endl;
        _gcsaddr = {};
        _gcsaddr.sin_family = AF_INET;
        if (inet_pton(AF_INET, gamecontrol_ip.c_str(), &_gcsaddr.sin_addr) != 1 ||
            _gcsaddr.sin_addr.s_addr == htonl(INADDR_ANY))
        {
            throw std::invalid_argument(
                "game_control_ip must be a valid, non-zero IPv4 address: " + gamecontrol_ip);
        }
        _gcsaddr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);

        _unicast_gamecontrol_flag = true;
        _gamecontrol_unicast_thread = std::thread([this](){ this->unicastToGameController(); });
    }
    catch(const std::exception& e)
    {
        if (_gc_send_socket >= 0)
        {
            close(_gc_send_socket);
            _gc_send_socket = -1;
        }
        std::cerr << e.what() << '\n';
    }
}

void BrainCommunication::clearupGameControllerUnicast()
{
    _unicast_gamecontrol_flag = false;
    if (_gamecontrol_unicast_thread.joinable())
    {
        _gamecontrol_unicast_thread.join();
    }
    if (_gc_send_socket >= 0)
    {
        close(_gc_send_socket);
        _gc_send_socket = -1;
        cout << RED_CODE << format("GameControl send socket has been closed.")
            << RESET_CODE << endl;
    }
}

void BrainCommunication::initDiscoveryBroadcast()
{
    try
    {
        _discovery_send_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_discovery_send_socket < 0)
        {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // Enable broadcasting.
        int broadcast = 1;
        if (setsockopt(_discovery_send_socket, SOL_SOCKET, SO_BROADCAST, 
                    &broadcast, sizeof(broadcast)) < 0)
        {
            cout << RED_CODE << format("Failed to set SO_BROADCAST: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // Configure the broadcast address.
        _saddr.sin_family = AF_INET;
        _saddr.sin_addr.s_addr = INADDR_BROADCAST;  // 255.255.255.255
        _saddr.sin_port = htons(_discovery_udp_port);

        _broadcast_discovery_flag = true;
        _discovery_broadcast_thread = std::thread([this](){ this->broadcastDiscovery(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", rerun::TextLog(format("Failed to initialize discovery broadcast: %s", e.what())));
    }
    
}

void BrainCommunication::clearupDiscoveryBroadcast()
{
    _broadcast_discovery_flag = false;
    if (_discovery_send_socket >= 0)
    {
        close(_discovery_send_socket);
        _discovery_send_socket = -1;
        cout << RED_CODE << format("Discovery send socket has been closed.")
            << RESET_CODE << endl;
    }

    if (_discovery_broadcast_thread.joinable())
    {
        _discovery_broadcast_thread.join();
    }
}

void BrainCommunication::initDiscoveryReceiver()
{
    try
    {
        _discovery_recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_discovery_recv_socket < 0)
        {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // Allow address reuse.
        int reuse = 1;
        if (setsockopt(_discovery_recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
        {
            cout << RED_CODE << format("Failed to set SO_REUSEADDR: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);  // Receive from all network interfaces.
        addr.sin_port = htons(_discovery_udp_port);
        
        if (bind(_discovery_recv_socket, (sockaddr *)&addr, sizeof(addr)) < 0)
        {
            cout << RED_CODE << format("bind failed: %s (port=%d)", strerror(errno), _discovery_udp_port)
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        cout << GREEN_CODE << format("Listening for UDP broadcast on port %d", _discovery_udp_port)
            << RESET_CODE << endl;

        _receive_discovery_flag = true;
        _discovery_recv_thread = std::thread([this](){ this->spinDiscoveryReceiver(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", rerun::TextLog(format("Failed to initialize discovery receiver: %s", e.what())));
    }
}

void BrainCommunication::clearupDiscoveryReceiver()
{
    _receive_discovery_flag = false;
    if (_discovery_recv_socket >= 0)
    {
        close(_discovery_recv_socket);
        _discovery_recv_socket = -1;
        cout << RED_CODE << format("Communication receive socket has been closed.")
            << RESET_CODE << endl;
    }
    if (_discovery_recv_thread.joinable())
    {
        _discovery_recv_thread.join();
    }
}

void BrainCommunication::unicastToGameController() {
    while (_unicast_gamecontrol_flag)
    {
        gc_return_data = brain->gameControllerReturnSnapshot();

        int ret = sendto(_gc_send_socket, &gc_return_data, sizeof(gc_return_data), 0, (sockaddr *)&_gcsaddr, sizeof(_gcsaddr));
        if (ret != static_cast<int>(sizeof(gc_return_data)))
        {
            char target[INET_ADDRSTRLEN]{};
            inet_ntop(AF_INET, &_gcsaddr.sin_addr, target, sizeof(target));
            cout << RED_CODE << format(
                "gc sendto failed: target=%s:%u sent=%d expected=%zu error=%s",
                target, ntohs(_gcsaddr.sin_port), ret, sizeof(gc_return_data), strerror(errno))
                << RESET_CODE << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(BROADCAST_GAME_CONTROL_INTERVAL_MS));
    }
}

void BrainCommunication::broadcastDiscovery() {
    while (_broadcast_discovery_flag)
    {
        TeamDiscoveryMsg msg;

        msg.communicationId = _discovery_msg_id++;
        msg.teamId = brain->config->teamId;
        msg.playerId = brain->config->playerId;
        // // msg.playerRole = tree->getEntry<string>("player_role");
        // msg.isAlive = !brain->tree->getEntry<bool>("gc_is_under_penalty");
        // msg.ballDetected = brain->data->ballDetected;
        // msg.ballRange = brain->data->ball.range;
        // msg.ballPosToField = brain->data->ball.posToField;
        // msg.robotPoseToField = brain->data->robotPoseToField;

        int ret = sendto(_discovery_send_socket, &msg, sizeof(msg), 0, (sockaddr *)&_saddr, sizeof(_saddr));
        if (ret < 0)
        {
            cout << RED_CODE << format(
                "discovery broadcast sendto failed: target=255.255.255.255:%u error=%s",
                ntohs(_saddr.sin_port), strerror(errno))
                << RESET_CODE << endl;
        }
        // cout << GREEN_CODE << format("broadcastDiscovery: %d", msg.communicationId)
            // << RESET_CODE << endl;
        brain->data->discoveryMsgId = msg.communicationId;
        brain->data->discoveryMsgTime = brain->get_clock()->now();
        this_thread::sleep_for(chrono::milliseconds(BROADCAST_DISCOVERY_INTERVAL_MS));
    } 
}

void BrainCommunication::spinDiscoveryReceiver() {    
    sockaddr_in addr{};
    socklen_t addr_len = sizeof(addr);

    TeamDiscoveryMsg msg;

    while (_receive_discovery_flag) {

        ssize_t len = recvfrom(_discovery_recv_socket, &msg, sizeof(msg), 0, (sockaddr *)&addr, &addr_len);

        if (len < 0)
        {
            cout << RED_CODE << format("receiving UDP message failed: %s", strerror(errno))
                << RESET_CODE << endl;
            continue;
        }

        if (len != sizeof(TeamDiscoveryMsg)) {
            cout << YELLOW_CODE << format("received TeamDiscoveryMsg packet with wrong size: %ld, expected: %ld", len, sizeof(TeamDiscoveryMsg))
                << RESET_CODE << endl;
            continue;
        }

        if (msg.validation != VALIDATION_DISCOVERY) { // fail to pass validation
            cout << RED_CODE << format("received TeamDiscoveryMsg packet with invalid validation: %d", msg.validation)
                << RESET_CODE << endl;
            continue;
        } 

        if (msg.teamId != brain->config->teamId) { // Ignore messages from other teams.
            cout << YELLOW_CODE << format("Received message from team %d, expected team %d", msg.teamId, brain->config->teamId)
                << RESET_CODE << endl;
            continue;
        }

        if (msg.playerId < 1 ||
            msg.playerId > brain->config->numOfPlayers) {
            cout << YELLOW_CODE << format(
                "Received discovery with invalid playerId: %d",
                msg.playerId) << RESET_CODE << endl;
            continue;
        }

        if (msg.playerId == brain->config->playerId) {  // Ignore this robot's messages.
            // Process this robot's message.
            // cout << YELLOW_CODE <<  format(
            //     "discoveryID: %d, teamId:%d, playerId: %d, address: %s:%d",
            //     msg.communicationId, msg.teamId, msg.playerId, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port))
            //     << RESET_CODE << endl;
            continue;
        } else {
            // Process a teammate message.
            // cout << GREEN_CODE <<  format(
            //     "discoveryID: %d, teamId:%d, playerId: %d, address: %s:%d",
            //     msg.communicationId, msg.teamId, msg.playerId, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port))
            //     << RESET_CODE << endl;
            
            auto time_now = brain->get_clock()->now();
            std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
            _teammate_addresses[addr.sin_addr.s_addr] = {
                addr.sin_addr.s_addr,
                msg.playerId,
                time_now
            };
        }
    }
}

void BrainCommunication::cleanupExpiredTeammates() {
    std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);    
    for (auto it = _teammate_addresses.begin(); it != _teammate_addresses.end();) {
        auto timeDiff = this->brain->get_clock()->now().nanoseconds() - it->second.lastUpdate.nanoseconds();
        if (timeDiff > TEAMMATE_TIMEOUT_MS * 1e6) {
            cout << YELLOW_CODE << format("Teammate id %d timed out", it->second.playerId) 
                 << RESET_CODE << endl;
            it = _teammate_addresses.erase(it);
        } else {
            ++it;
        }
    }
}

void BrainCommunication::initCommunicationUnicast() {
    try
    {
        _unicast_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_unicast_socket < 0) {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error("Failed to create unicast socket");
        }

        _unicast_saddr.sin_family = AF_INET;
        _unicast_saddr.sin_port = htons(_unicast_udp_port);

        _unicast_communication_flag = true;
        _unicast_thread = std::thread([this](){ this->unicastCommunication(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", rerun::TextLog(format("Failed to initialize unicast communication: %s", e.what())));
    }
    
}

int BrainCommunication::teamCommunicationIntervalMs() const
{
    double rateHz = DEFAULT_TEAM_COMMUNICATION_RATE_HZ;
    brain->get_parameter("communication.team_broadcast_rate_hz", rateHz);
    if (!std::isfinite(rateHz)) {
        rateHz = DEFAULT_TEAM_COMMUNICATION_RATE_HZ;
    }
    rateHz = std::clamp(rateHz, 0.1, MAX_TEAM_COMMUNICATION_RATE_HZ);
    return std::max(50, static_cast<int>(std::lround(1000.0 / rateHz)));
}

void BrainCommunication::unicastCommunication() {
    auto log = [=](string msg) {
        if (!brain->log->shouldLog(
                "communication_send_debug", brain->config->rerunLogDebugHz))
            return;
        brain->log->setTimeNow();
        brain->log->log("debug/sendMsg", rerun::TextLog(msg));
    };
    while (_unicast_communication_flag) {
        cleanupExpiredTeammates();
        TeamCommunicationMsg msg{};
        const uint32_t sequence = _team_communication_sequence++;
        msg.validation = VALIDATION_COMMUNICATION;
        msg.protocolVersion = TEAM_COMMUNICATION_PROTOCOL_VERSION;
        msg.payloadBytes = static_cast<uint16_t>(sizeof(TeamCommunicationMsg));
        msg.bootId = _boot_id;
        msg.sequence = sequence;
        msg.communicationId = static_cast<int>(sequence & 0x7FFFFFFFU);
        msg.teamId = brain->config->teamId;
        msg.playerId = brain->config->playerId;
        msg.playerStartRole = brain->config->playerRole == "striker" ? 1 : 2;
        TeamOutboundSnapshot snapshot;
        {
            std::lock_guard<std::mutex> snapshotLock(
                brain->data->teamOutboundMutex);
            snapshot = brain->data->tmOutboundSnapshot;
        }
        msg.playerRole = snapshot.playerRole;
        msg.isAlive = snapshot.isAlive;
        msg.isLead = snapshot.isLead;
        msg.ballDetected = snapshot.ballDetected;
        msg.ballLocationKnown = snapshot.ballLocationKnown;
        msg.ballConfidence = snapshot.ballConfidence;
        msg.ballRange = snapshot.ballRange;
        msg.cost = snapshot.cost;
        msg.ballPosToField = snapshot.ballPosToField;
        msg.robotPoseToField = snapshot.robotPoseToField;
        msg.kickDir = snapshot.kickDir;
        msg.thetaRb = snapshot.thetaRb;
        msg.ballOwnerId = snapshot.ballOwnerId;
        msg.leaderTerm = snapshot.leaderTerm;
        msg.formationOwnerId = snapshot.formationOwnerId;
        msg.formationRevision = snapshot.formationRevision;
        msg.formationShadowSide = snapshot.formationShadowSide;
        for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
            msg.assistSlots[i] = static_cast<int>(snapshot.assistSlots[i]);
        }
        msg.assistSlot = static_cast<int>(snapshot.assistSlot);
        msg.assistPhase = static_cast<int>(snapshot.assistPhase);
        msg.assistTarget = snapshot.assistTarget;
        msg.kickoffActive = snapshot.kickoffActive;
        msg.kickoffOwnerId = snapshot.kickoffOwnerId;
        msg.kickoffOwnerTerm = snapshot.kickoffOwnerTerm;
        msg.freeKickProposerBootId = snapshot.freeKickProposerBootId;
        msg.freeKickRefereePacketNumber =
            snapshot.freeKickRefereePacketNumber;
        msg.freeKickEventId = snapshot.freeKickEventId;
        msg.freeKickLeaderTerm = snapshot.freeKickLeaderTerm;
        msg.freeKickType = static_cast<int>(snapshot.freeKickType);
        msg.freeKickPhase = static_cast<int>(snapshot.freeKickPhase);
        msg.freeKickOurRestart = snapshot.freeKickOurRestart;
        msg.freeKickKickerId = snapshot.freeKickKickerId;
        msg.freeKickReceiverId = snapshot.freeKickReceiverId;
        msg.freeKickBall = snapshot.freeKickBall;
        msg.freeKickPassTarget = snapshot.freeKickPassTarget;
        for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
            msg.freeKickAssistSlots[i] =
                static_cast<int>(snapshot.freeKickAssistSlots[i]);
        }
        msg.freeKickExecutionElapsedMs =
            snapshot.freeKickExecutionElapsedMs;
        msg.cmdId = snapshot.cmdId;
        msg.cmd = snapshot.cmd;
        log(format("ImAlive: %d, ImLead: %d, myCost: %.1f, myCmdId: %d, myCmd: %d", msg.isAlive, msg.isLead, msg.cost, msg.cmdId, msg.cmd));

        {
            std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
            for (auto it = _teammate_addresses.begin(); it != _teammate_addresses.end(); ++it) {
                auto ip = it->second.ip;

                // cout << GREEN_CODE << format("unicastCommunication to %s", inet_ntoa(*(in_addr *)&ip))
                //     << RESET_CODE << endl;
                brain->data->tmIP = inet_ntoa(*(in_addr *)&ip);
                brain->data->sendId = msg.communicationId;
                brain->data->sendTime = brain->get_clock()->now();

                _unicast_saddr.sin_addr.s_addr = ip;
                int ret = sendto(_unicast_socket, &msg, sizeof(msg), 0, (sockaddr *)&_unicast_saddr, sizeof(_unicast_saddr));
                if (ret < 0) {
                    char target[INET_ADDRSTRLEN]{};
                    inet_ntop(AF_INET, &_unicast_saddr.sin_addr, target, sizeof(target));
                    cout << RED_CODE << format(
                        "team unicast sendto failed: target=%s:%u player=%d error=%s",
                        target, ntohs(_unicast_saddr.sin_port), it->second.playerId,
                        strerror(errno))
                        << RESET_CODE << endl;
                }
            }
        }
        this_thread::sleep_for(
            chrono::milliseconds(teamCommunicationIntervalMs()));
    }
}

void BrainCommunication::clearupCommunicationUnicast() {
    _unicast_communication_flag = false;
    if (_unicast_socket >= 0) {
        close(_unicast_socket);
        _unicast_socket = -1;
        cout << RED_CODE << format("Communication send socket has been closed.")
            << RESET_CODE << endl;
    }

    if (_unicast_thread.joinable()) {
        _unicast_thread.join();
    }
}

void BrainCommunication::initCommunicationReceiver() {
    try
    {
        _communication_recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_communication_recv_socket < 0) {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // Allow address reuse.
        int reuse = 1;
        if (setsockopt(_communication_recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
        {
            cout << RED_CODE << format("Failed to set SO_REUSEADDR: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(_unicast_udp_port);
        
        if (bind(_communication_recv_socket, (sockaddr *)&addr, sizeof(addr)) < 0) {
            cout << RED_CODE << format("bind failed: %s (port=%d)", strerror(errno), _unicast_udp_port)
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        _receive_communication_flag = true;
        _communication_recv_thread = std::thread([this](){ this->spinCommunicationReceiver(); });
    
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", rerun::TextLog(format("Failed to initialize communication receiver: %s", e.what())));
    }
}

void BrainCommunication::spinCommunicationReceiver() {
    auto log = [=](string msg) {
        if (!brain->log->shouldLog(
                "communication_receive_debug", brain->config->rerunLogDebugHz))
            return;
        brain->log->setTimeNow();
        brain->log->log("debug/receiveMsg", rerun::TextLog(msg));
    };
    sockaddr_in addr{};
    socklen_t addr_len = sizeof(addr);

    TeamCommunicationMsg msg{};

    while (_receive_communication_flag) {

        ssize_t len = recvfrom(_communication_recv_socket, &msg, sizeof(msg), 0, (sockaddr *)&addr, &addr_len);

        if (len < 0) {
            cout << RED_CODE << format("receiving UDP message failed: %s", strerror(errno))
                << RESET_CODE << endl;
            continue;
        }

        if (len != sizeof(TeamCommunicationMsg)) {
            cout << YELLOW_CODE << format("received TeamCommunicationMsg packet with wrong size: %ld, expected: %ld", len, sizeof(TeamCommunicationMsg))
                << RESET_CODE << endl;
            continue;
        }

        if (msg.validation != VALIDATION_COMMUNICATION) { // fail to pass validation
            cout << RED_CODE << format("received TeamCommunicationMsg packet with invalid validation: %d", msg.validation)
                << RESET_CODE << endl;
            continue;
        }

        if (msg.protocolVersion != TEAM_COMMUNICATION_PROTOCOL_VERSION ||
            msg.payloadBytes != sizeof(TeamCommunicationMsg)) {
            cout << YELLOW_CODE << format(
                "received unsupported team protocol: version=%u bytes=%u",
                static_cast<unsigned>(msg.protocolVersion),
                static_cast<unsigned>(msg.payloadBytes)) << RESET_CODE << endl;
            continue;
        }

        if (msg.teamId != brain->config->teamId) { // Ignore messages from other teams.
            cout << YELLOW_CODE << format("Received message from team %d, expected team %d", msg.teamId, brain->config->teamId)
                << RESET_CODE << endl;
            continue;
        }

        if (msg.playerId < 1 ||
            msg.playerId > brain->config->numOfPlayers ||
            msg.playerId > MAX_NUM_PLAYERS) {
            cout << YELLOW_CODE << format(
                "Received communication with invalid playerId: %d",
                msg.playerId) << RESET_CODE << endl;
            continue;
        }

        if (msg.playerId == brain->config->playerId) {  // Ignore this robot's messages.
            // Process this robot's message.
            cout << CYAN_CODE <<  format(
                "communicationId: %d, alive: %d, ballDetected: %d ballRange: %.2f playerId: %d",
                msg.communicationId, msg.isAlive, msg.ballDetected, msg.ballRange, msg.playerId)
                << RESET_CODE << endl;
            brain->data->sendId = msg.communicationId;
            brain->data->sendTime = brain->get_clock()->now();
            continue;
        } 

        // Otherwise process a teammate message.
        // cout << GREEN_CODE <<  format(
        //     "communicationId: %d, alive: %d, ballDetected: %d ballRange: %.2f playerId: %d",
        //     msg.communicationId, msg.isAlive, msg.ballDetected, msg.ballRange, msg.playerId)
        //     << RESET_CODE << endl;
        const auto tmIdx = msg.playerId - 1;
        bool sourceIdentityMismatch = false;
        {
            std::lock_guard<std::mutex> addressLock(
                _teammate_addresses_mutex);
            const auto source = _teammate_addresses.find(
                addr.sin_addr.s_addr);
            sourceIdentityMismatch =
                source != _teammate_addresses.end() &&
                source->second.playerId != msg.playerId;
        }
        if (sourceIdentityMismatch) {
            cout << YELLOW_CODE << format(
                "Ignored communication whose source address belongs to another player: %d",
                msg.playerId) << RESET_CODE << endl;
            continue;
        }

        const auto finitePoint = [](const Point &point) {
            return std::isfinite(point.x) &&
                   std::isfinite(point.y) &&
                   std::isfinite(point.z);
        };
        const auto finitePose = [](const Pose2D &pose) {
            return std::isfinite(pose.x) &&
                   std::isfinite(pose.y) &&
                   std::isfinite(pose.theta);
        };
        const bool semanticFieldsValid =
            msg.bootId != 0 &&
            (msg.playerRole == 1 || msg.playerRole == 2) &&
            (msg.playerStartRole == 1 || msg.playerStartRole == 2) &&
            msg.ballOwnerId >= 0 &&
            msg.ballOwnerId <= brain->config->numOfPlayers &&
            msg.formationOwnerId >= 0 &&
            msg.formationOwnerId <= brain->config->numOfPlayers &&
            msg.kickoffOwnerId >= 0 &&
            msg.kickoffOwnerId <= brain->config->numOfPlayers &&
            msg.freeKickRefereePacketNumber <= UINT8_MAX &&
            msg.freeKickType >= 0 &&
            msg.freeKickType <= 2 &&
            msg.freeKickPhase >= 0 &&
            msg.freeKickPhase <= 6 &&
            msg.freeKickKickerId >= 0 &&
            msg.freeKickKickerId <= brain->config->numOfPlayers &&
            msg.freeKickReceiverId >= 0 &&
            msg.freeKickReceiverId <= brain->config->numOfPlayers &&
            msg.formationShadowSide >= -1 &&
            msg.formationShadowSide <= 1 &&
            std::isfinite(msg.ballConfidence) &&
            std::isfinite(msg.ballRange) && msg.ballRange >= 0.0 &&
            std::isfinite(msg.cost) &&
            finitePoint(msg.ballPosToField) &&
            finitePose(msg.robotPoseToField) &&
            std::isfinite(msg.kickDir) &&
            std::isfinite(msg.thetaRb) &&
            std::isfinite(msg.assistTarget.x) &&
            std::isfinite(msg.assistTarget.y) &&
            finitePoint(msg.freeKickBall) &&
            std::isfinite(msg.freeKickPassTarget.x) &&
            std::isfinite(msg.freeKickPassTarget.y);
        if (!semanticFieldsValid) {
            cout << YELLOW_CODE << format(
                "Ignored malformed tactical packet from player %d",
                msg.playerId) << RESET_CODE << endl;
            continue;
        }

        log(format("TMID: %.d, alive: %d, lead: %d, cost: %.1f, CmdId: %d, Cmd: %d", msg.playerId, msg.isAlive, msg.isLead, msg.cost, msg.cmdId, msg.cmd));

        std::lock_guard<std::mutex> teamStatusLock(
            brain->data->teamStatusMutex);
        TMStatus &tmStatus = brain->data->tmStatus[tmIdx];
        const auto clearPendingBoot = [&]() {
            tmStatus.pendingBootId = 0;
            tmStatus.pendingBootSequence = 0;
            tmStatus.pendingBootPacketCount = 0;
            tmStatus.pendingBootFirstSeen = {};
        };
        if (msg.bootId != tmStatus.bootId &&
            std::find(
                tmStatus.retiredBootIds.begin(),
                tmStatus.retiredBootIds.end(),
                msg.bootId) != tmStatus.retiredBootIds.end()) {
            log(format(
                "ignored packet from retired boot of teammate %d",
                msg.playerId));
            continue;
        }
        if (tmStatus.bootId == msg.bootId) {
            if (!sequenceIsNewer(msg.sequence, tmStatus.sequence)) {
                log(format(
                    "ignored stale packet from teammate %d: sequence=%u, latest=%u",
                    msg.playerId,
                    static_cast<unsigned>(msg.sequence),
                    static_cast<unsigned>(tmStatus.sequence)));
                continue;
            }
            // Only a genuinely advancing packet proves that the current
            // process is still alive. A delayed duplicate must not erase a
            // nearly confirmed replacement boot.
            clearPendingBoot();
        } else if (tmStatus.bootId != 0) {
            const double streamTimeoutMs = std::max(
                100.0,
                brain->get_parameter(
                    "strategy.cooperation.tactical_packet_timeout_ms")
                    .as_double());
            if (brain->msecsSince(tmStatus.timeLastCom) <= streamTimeoutMs) {
                // Do not let one packet from an unknown boot retire a stream that is still fresh.
                clearPendingBoot();
                continue;
            }

            const auto now = std::chrono::steady_clock::now();
            const bool withinConfirmationWindow =
                tmStatus.pendingBootFirstSeen !=
                    std::chrono::steady_clock::time_point{} &&
                now - tmStatus.pendingBootFirstSeen <=
                    BOOT_CONFIRMATION_WINDOW;
            const bool sameCandidateWithinWindow =
                tmStatus.pendingBootId == msg.bootId &&
                withinConfirmationWindow;
            if (sameCandidateWithinWindow) {
                if (!sequenceIsNewer(
                        msg.sequence, tmStatus.pendingBootSequence)) {
                    log(format(
                        "ignored stale pending-boot packet from teammate %d: sequence=%u, latest=%u",
                        msg.playerId,
                        static_cast<unsigned>(msg.sequence),
                        static_cast<unsigned>(
                            tmStatus.pendingBootSequence)));
                    continue;
                }
                tmStatus.pendingBootSequence = msg.sequence;
                tmStatus.pendingBootPacketCount++;
            } else {
                tmStatus.pendingBootId = msg.bootId;
                tmStatus.pendingBootSequence = msg.sequence;
                tmStatus.pendingBootPacketCount = 1;
                tmStatus.pendingBootFirstSeen = now;
            }
            if (tmStatus.pendingBootPacketCount <
                BOOT_CONFIRMATION_PACKET_COUNT) {
                continue;
            }

            tmStatus.retiredBootIds.push_back(tmStatus.bootId);
            log(format(
                "confirmed teammate %d boot transition after %u packets",
                msg.playerId,
                static_cast<unsigned>(
                    tmStatus.pendingBootPacketCount)));
            clearPendingBoot();
        }
        
        tmStatus.role = msg.playerRole == 1 ? "striker" :
            (msg.playerRole == 2 ? "goal_keeper" : "not initialized");
        tmStatus.startRole = msg.playerStartRole == 1 ? "striker" :
            (msg.playerStartRole == 2 ? "goal_keeper" : "not initialized");
        tmStatus.isAlive = msg.isAlive;
        tmStatus.ballDetected = msg.ballDetected;
        tmStatus.ballLocationKnown = msg.ballLocationKnown;
        tmStatus.ballConfidence = msg.ballConfidence;
        tmStatus.ballRange = msg.ballRange;
        tmStatus.cost = msg.cost;
        tmStatus.isLead = msg.isLead;
        tmStatus.ballPosToField = msg.ballPosToField;
        tmStatus.robotPoseToField = msg.robotPoseToField;
        tmStatus.kickDir = msg.kickDir;
        tmStatus.thetaRb = msg.thetaRb;
        tmStatus.bootId = msg.bootId;
        tmStatus.sequence = msg.sequence;
        tmStatus.ballOwnerId = msg.ballOwnerId;
        tmStatus.leaderTerm = msg.leaderTerm;
        tmStatus.formationOwnerId = msg.formationOwnerId;
        tmStatus.formationRevision = msg.formationRevision;
        tmStatus.formationShadowSide =
            std::abs(msg.formationShadowSide) == 1
            ? msg.formationShadowSide
            : 0;
        for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
            const int slot = msg.assistSlots[i];
            tmStatus.assistSlots[i] =
                slot >= static_cast<int>(AssistSlot::NONE) &&
                slot <= static_cast<int>(AssistSlot::WIDE_OUTLET)
                ? slot
                : static_cast<int>(AssistSlot::NONE);
        }
        tmStatus.assistSlot =
            msg.assistSlot >= static_cast<int>(AssistSlot::NONE) &&
            msg.assistSlot <= static_cast<int>(AssistSlot::WIDE_OUTLET)
            ? static_cast<AssistSlot>(msg.assistSlot)
            : AssistSlot::NONE;
        tmStatus.assistPhase =
            msg.assistPhase >= static_cast<int>(AssistPhase::HOLD) &&
            msg.assistPhase <= static_cast<int>(AssistPhase::TRANSIT_SLOT)
            ? static_cast<AssistPhase>(msg.assistPhase)
            : AssistPhase::HOLD;
        tmStatus.assistTarget = msg.assistTarget;
        tmStatus.kickoffActive = msg.kickoffActive;
        tmStatus.kickoffOwnerId = msg.kickoffOwnerId;
        tmStatus.kickoffOwnerTerm = msg.kickoffOwnerTerm;
        tmStatus.freeKickProposerBootId = msg.freeKickProposerBootId;
        tmStatus.freeKickRefereePacketNumber =
            msg.freeKickRefereePacketNumber;
        tmStatus.freeKickEventId = msg.freeKickEventId;
        tmStatus.freeKickLeaderTerm = msg.freeKickLeaderTerm;
        tmStatus.freeKickType = msg.freeKickType;
        tmStatus.freeKickPhase = msg.freeKickPhase;
        tmStatus.freeKickOurRestart = msg.freeKickOurRestart;
        tmStatus.freeKickKickerId = msg.freeKickKickerId;
        tmStatus.freeKickReceiverId = msg.freeKickReceiverId;
        tmStatus.freeKickBall = msg.freeKickBall;
        tmStatus.freeKickPassTarget = msg.freeKickPassTarget;
        for (int i = 0; i < MAX_NUM_PLAYERS; ++i) {
            const int slot = msg.freeKickAssistSlots[i];
            tmStatus.freeKickAssistSlots[i] =
                slot >= static_cast<int>(AssistSlot::NONE) &&
                slot <= static_cast<int>(AssistSlot::WIDE_OUTLET)
                ? slot
                : static_cast<int>(AssistSlot::NONE);
        }
        tmStatus.freeKickExecutionElapsedMs =
            msg.freeKickExecutionElapsedMs;
        tmStatus.timeLastCom = brain->get_clock()->now();
        tmStatus.cmd = msg.cmd;
        tmStatus.cmdId = msg.cmdId;
        brain->data->receiveId[tmIdx] = msg.communicationId;
        brain->data->receiveTime[tmIdx] = tmStatus.timeLastCom;

        // Check for a new command.
        if (msg.cmdId > brain->data->tmCmdId) {
            brain->data->tmCmdId = msg.cmdId;
            brain->data->tmReceivedCmd = msg.cmd;
            brain->data->tmLastCmdChangeTime = brain->get_clock()->now();
            log(format("Received new command from teammate %d: %d", msg.playerId, msg.cmd));
        }

    }
}

void BrainCommunication::clearupCommunicationReceiver() {
    _receive_communication_flag = false;
    if (_communication_recv_socket >= 0) {
        close(_communication_recv_socket);
        _communication_recv_socket = -1;
        cout << RED_CODE << format("Communication receive socket has been closed.")
            << RESET_CODE << endl;
    }
    if (_communication_recv_thread.joinable()) {
        _communication_recv_thread.join();
    }
}
