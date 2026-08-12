#include "obstacle_avoidance_logger.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

namespace {

std::string jsonEscape(const std::string &value)
{
    std::ostringstream escaped;
    for (const unsigned char character : value) {
        switch (character) {
        case '"': escaped << "\\\""; break;
        case '\\': escaped << "\\\\"; break;
        case '\n': escaped << "\\n"; break;
        case '\r': escaped << "\\r"; break;
        case '\t': escaped << "\\t"; break;
        default:
            if (character < 0x20) {
                escaped << "\\u" << std::hex << std::setw(4)
                        << std::setfill('0') << static_cast<int>(character)
                        << std::dec << std::setfill(' ');
            } else {
                escaped << character;
            }
        }
    }
    return escaped.str();
}

void writeNumber(std::ostream &stream, double value)
{
    if (std::isfinite(value)) stream << value;
    else stream << "null";
}

int64_t systemTimeNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

} // namespace

ObstacleAvoidanceLogger::ObstacleAvoidanceLogger(
    std::string filePath,
    double maximumHz,
    std::uintmax_t maximumBytes,
    std::size_t maximumFiles)
    : filePath_(std::move(filePath)),
      maximumHz_(maximumHz),
      maximumBytes_(maximumBytes),
      maximumFiles_(maximumFiles)
{
    if (filePath_.empty()) {
        error_ = "empty log path";
        return;
    }
    const std::filesystem::path path(filePath_);
    std::error_code error;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path(), error);
        if (error) {
            error_ = "cannot create log directory: " + error.message();
            return;
        }
    }
    bool sessionAlreadyWritten = false;
    if (maximumBytes_ > 0 && std::filesystem::exists(path, error) &&
        !error && std::filesystem::file_size(path, error) >= maximumBytes_ &&
        !error) {
        if (!open(true) || !rotateIfNeeded()) return;
        sessionAlreadyWritten = true;
    } else if (!open(true)) {
        return;
    }
    if (!sessionAlreadyWritten) writeSessionRecord();
    stream_.flush();
    enabled_ = static_cast<bool>(stream_);
    if (!enabled_ && error_.empty()) error_ = "failed to write session record";
}

ObstacleAvoidanceLogger::~ObstacleAvoidanceLogger()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stream_.is_open()) return;
    stream_ << "{\"type\":\"shutdown\",\"system_time_ns\":"
            << systemTimeNs() << "}\n";
    stream_.flush();
    stream_.close();
    enabled_ = false;
}

bool ObstacleAvoidanceLogger::enabled() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
}

const std::string &ObstacleAvoidanceLogger::filePath() const
{
    return filePath_;
}

const std::string &ObstacleAvoidanceLogger::error() const
{
    return error_;
}

bool ObstacleAvoidanceLogger::shouldLog(
    const std::string &source,
    const std::string &reason)
{
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) return false;
    const auto state = sourceStates_.find(source);
    if (state == sourceStates_.end() || state->second.lastReason != reason) {
        return true;
    }
    return !std::isfinite(maximumHz_) || maximumHz_ <= 0.0 ||
        state->second.lastWrite == std::chrono::steady_clock::time_point{} ||
        now - state->second.lastWrite >=
            std::chrono::duration<double>(1.0 / maximumHz_);
}

bool ObstacleAvoidanceLogger::open(bool append)
{
    if (stream_.is_open()) stream_.close();
    const auto mode = std::ios::out |
        (append ? std::ios::app : std::ios::trunc);
    stream_.open(filePath_, mode);
    if (!stream_.is_open()) {
        error_ = "cannot open log file";
        enabled_ = false;
        return false;
    }
    stream_ << std::setprecision(10);
    enabled_ = true;
    return true;
}

bool ObstacleAvoidanceLogger::rotateIfNeeded()
{
    if (maximumBytes_ == 0) return true;
    stream_.flush();
    std::error_code error;
    const std::filesystem::path base(filePath_);
    const auto size = std::filesystem::file_size(base, error);
    if (error || size < maximumBytes_) return true;
    stream_.close();

    if (maximumFiles_ == 0) return open(false);
    for (std::size_t index = maximumFiles_; index > 0; --index) {
        const std::filesystem::path source = index == 1
            ? base
            : std::filesystem::path(filePath_ + "." +
                  std::to_string(index - 1));
        const std::filesystem::path destination(
            filePath_ + "." + std::to_string(index));
        error.clear();
        std::filesystem::remove(destination, error);
        error.clear();
        if (std::filesystem::exists(source, error) && !error) {
            std::filesystem::rename(source, destination, error);
            if (error) {
                error_ = "cannot rotate log file: " + error.message();
                return open(true);
            }
        }
    }
    if (!open(false)) return false;
    writeSessionRecord();
    return true;
}

void ObstacleAvoidanceLogger::writeSessionRecord()
{
    stream_ << "{\"type\":\"session\",\"schema_version\":2"
            << ",\"system_time_ns\":" << systemTimeNs()
            << ",\"maximum_hz\":";
    writeNumber(stream_, maximumHz_);
    stream_ << ",\"maximum_bytes\":" << maximumBytes_
            << ",\"maximum_files\":" << maximumFiles_ << "}\n";
}

void ObstacleAvoidanceLogger::log(
    const ObstacleAvoidanceLogRecord &record)
{
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) return;

    SourceState &state = sourceStates_[record.source];
    const bool reasonChanged = state.lastReason != record.reason;
    if (!reasonChanged && std::isfinite(maximumHz_) && maximumHz_ > 0.0 &&
        state.lastWrite != std::chrono::steady_clock::time_point{} &&
        now - state.lastWrite < std::chrono::duration<double>(1.0 / maximumHz_)) {
        return;
    }

    const bool blocked = record.reason == "blocked_stop" ||
        record.reason == "blocked_turn";
    if (blocked && !state.blocked) {
        state.blocked = true;
        state.blockedSince = now;
    } else if (!blocked) {
        state.blocked = false;
        state.blockedSince = {};
    }
    const double blockedDurationMs = state.blocked
        ? std::chrono::duration<double, std::milli>(
              now - state.blockedSince).count()
        : 0.0;

    if (!rotateIfNeeded()) return;
    writeRecord(record, blockedDurationMs);
    stream_.flush();
    if (!stream_) {
        error_ = "failed while writing log file";
        enabled_ = false;
        return;
    }
    state.lastReason = record.reason;
    state.lastWrite = now;
}

void ObstacleAvoidanceLogger::writeRecord(
    const ObstacleAvoidanceLogRecord &record,
    double blockedDurationMs)
{
    stream_ << "{\"type\":\"avoidance\",\"system_time_ns\":"
            << systemTimeNs() << ",\"ros_time_ns\":" << record.rosTimeNs
            << ",\"build_version\":\"" << jsonEscape(record.buildVersion) << '"'
            << ",\"source\":\"" << jsonEscape(record.source)
            << "\",\"reason\":\"" << jsonEscape(record.reason) << '"';
    const auto number = [this](const char *name, double value) {
        stream_ << ",\"" << name << "\":";
        writeNumber(stream_, value);
    };
    number("robot_x", record.robotX);
    number("robot_y", record.robotY);
    number("robot_theta", record.robotTheta);
    number("target_x", record.targetX);
    number("target_y", record.targetY);
    number("target_range", record.targetRange);
    number("target_angle", record.targetAngle);
    stream_ << ",\"plan_direct\":" << (record.planDirect ? "true" : "false")
            << ",\"plan_has_path\":" << (record.planHasPath ? "true" : "false")
            << ",\"escaping_overlap\":"
            << (record.escapingOverlap ? "true" : "false")
            << ",\"overlap_count\":" << record.overlappingObstacleCount;
    number("waypoint_x", record.waypointX);
    number("waypoint_y", record.waypointY);
    number("path_min_clearance", record.pathMinimumClearance);
    number("path_start_clearance", record.pathStartClearance);
    number("escape_clearance_gain", record.escapeClearanceGain);
    number("avoidance_side", record.avoidanceSide);
    number("safe_distance", record.safeDistance);
    number("path_clearance", record.pathClearance);
    number("self_radius", record.selfRadius);
    number("target_clearance", record.targetClearance);
    number("selected_angle", record.selectedAngle);
    number("selected_clearance", record.selectedClearance);
    number("requested_vx", record.requestedVx);
    number("requested_vy", record.requestedVy);
    number("requested_vtheta", record.requestedVtheta);
    number("sent_vx", record.sentVx);
    number("sent_vy", record.sentVy);
    number("sent_vtheta", record.sentVtheta);
    number("blocked_duration_ms", blockedDurationMs);
    stream_ << ",\"depth_count\":" << record.depthObstacleCount
            << ",\"ball_count\":" << record.ballObstacleCount
            << ",\"robot_count\":" << record.robotObstacleCount
            << ",\"raw_robot_detection_count\":"
            << record.rawRobotDetectionCount
            << ",\"merged_robot_detection_count\":"
            << record.mergedRobotDetectionCount
            << ",\"obstacles\":[";
    for (std::size_t index = 0; index < record.obstacles.size(); ++index) {
        const auto &obstacle = record.obstacles[index];
        if (index > 0) stream_ << ',';
        stream_ << "{\"type\":\"" << jsonEscape(obstacle.type) << '"';
        number("x", obstacle.x);
        number("y", obstacle.y);
        number("radius", obstacle.radius);
        number("confidence", obstacle.confidence);
        number("age_ms", obstacle.ageMs);
        stream_ << ",\"seen\":" << obstacle.seenCount
                << ",\"missed\":" << obstacle.missedCount
                << ",\"fallen\":" << (obstacle.fallen ? "true" : "false")
                << '}';
    }
    stream_ << ']'
            << ",\"depth_sample_step\":" << record.depthSampleStep
            << ",\"occupancy_threshold\":" << record.occupancyThreshold
            << ",\"camera_width\":" << record.cameraWidth
            << ",\"camera_height\":" << record.cameraHeight
            << ",\"depth_image_width\":" << record.depthImageWidth
            << ",\"depth_image_height\":" << record.depthImageHeight
            << ",\"depth_grid_width\":" << record.depthGridWidth
            << ",\"depth_grid_height\":" << record.depthGridHeight
            << ",\"depth_grid_occupied_cells\":"
            << record.depthGridOccupiedCells
            << ",\"depth_grid_max_occupancy\":"
            << record.depthGridMaxOccupancy
            << ",\"depth_grid_confirmed_cells\":"
            << record.depthGridConfirmedCells;
    number("obstacle_memory_msecs", record.obstacleMemoryMsecs);
    number("robot_merge_distance", record.robotMergeDistance);
    number("robot_minimum_distance", record.robotMinimumDistance);
    number("avoidance_max_speed", record.avoidanceMaximumSpeed);
    number("avoidance_min_vy", record.avoidanceMinimumVy);
    number("avoidance_reverse_speed", record.avoidanceReverseSpeed);
    stream_
            << ",\"depth_sampling_invalid\":"
            << (record.depthSamplingInvalid ? "true" : "false")
            << "}\n";
}
