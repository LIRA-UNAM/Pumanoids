#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

struct ObstacleAvoidanceLogObstacle
{
    std::string type;
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    double confidence = 0.0;
    double ageMs = 0.0;
    uint32_t seenCount = 0;
    uint32_t missedCount = 0;
    bool fallen = false;
};

struct ObstacleAvoidanceLogRecord
{
    int64_t rosTimeNs = 0;
    std::string source;
    std::string reason;
    double robotX = 0.0;
    double robotY = 0.0;
    double robotTheta = 0.0;
    double targetX = 0.0;
    double targetY = 0.0;
    double targetRange = 0.0;
    double targetAngle = 0.0;
    bool planDirect = true;
    bool planHasPath = true;
    bool escapingOverlap = false;
    std::size_t overlappingObstacleCount = 0;
    double waypointX = 0.0;
    double waypointY = 0.0;
    double pathMinimumClearance = 0.0;
    double pathStartClearance = 0.0;
    double escapeClearanceGain = 0.0;
    double avoidanceSide = 0.0;
    double safeDistance = 0.0;
    double pathClearance = 0.0;
    double selfRadius = 0.0;
    double targetClearance = 0.0;
    double selectedAngle = 0.0;
    double selectedClearance = 0.0;
    double requestedVx = 0.0;
    double requestedVy = 0.0;
    double requestedVtheta = 0.0;
    double sentVx = 0.0;
    double sentVy = 0.0;
    double sentVtheta = 0.0;
    std::size_t depthObstacleCount = 0;
    std::size_t ballObstacleCount = 0;
    std::size_t robotObstacleCount = 0;
    std::size_t rawRobotDetectionCount = 0;
    std::size_t mergedRobotDetectionCount = 0;
    int depthSampleStep = 0;
    int occupancyThreshold = 0;
    int cameraWidth = 0;
    int cameraHeight = 0;
    int depthImageWidth = 0;
    int depthImageHeight = 0;
    int depthGridWidth = 0;
    int depthGridHeight = 0;
    std::size_t depthGridOccupiedCells = 0;
    int depthGridMaxOccupancy = 0;
    std::size_t depthGridConfirmedCells = 0;
    double obstacleMemoryMsecs = 0.0;
    double robotMergeDistance = 0.0;
    double robotMinimumDistance = 0.0;
    double avoidanceMaximumSpeed = 0.0;
    double avoidanceMinimumVy = 0.0;
    double avoidanceReverseSpeed = 0.0;
    std::string buildVersion;
    bool depthSamplingInvalid = false;
    std::vector<ObstacleAvoidanceLogObstacle> obstacles;
};

class ObstacleAvoidanceLogger
{
public:
    ObstacleAvoidanceLogger(
        std::string filePath,
        double maximumHz,
        std::uintmax_t maximumBytes,
        std::size_t maximumFiles);
    ~ObstacleAvoidanceLogger();

    ObstacleAvoidanceLogger(const ObstacleAvoidanceLogger &) = delete;
    ObstacleAvoidanceLogger &operator=(const ObstacleAvoidanceLogger &) = delete;

    bool enabled() const;
    const std::string &filePath() const;
    const std::string &error() const;
    bool shouldLog(const std::string &source, const std::string &reason);
    void log(const ObstacleAvoidanceLogRecord &record);

private:
    struct SourceState
    {
        std::string lastReason;
        std::chrono::steady_clock::time_point lastWrite;
        std::chrono::steady_clock::time_point blockedSince;
        bool blocked = false;
    };

    bool open(bool append);
    bool rotateIfNeeded();
    void writeSessionRecord();
    void writeRecord(
        const ObstacleAvoidanceLogRecord &record,
        double blockedDurationMs);

    std::string filePath_;
    double maximumHz_ = 5.0;
    std::uintmax_t maximumBytes_ = 10 * 1024 * 1024;
    std::size_t maximumFiles_ = 3;
    std::ofstream stream_;
    std::string error_;
    bool enabled_ = false;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, SourceState> sourceStates_;
};
