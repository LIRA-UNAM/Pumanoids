#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <variant>

struct OdomDiagnosticPose
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct OdomDiagnosticVelocityCommand
{
    double requestedX = 0.0;
    double requestedY = 0.0;
    double requestedTheta = 0.0;
    double sentX = 0.0;
    double sentY = 0.0;
    double sentTheta = 0.0;
    int64_t rosTimeNs = 0;
};

struct OdomDiagnosticSample
{
    uint64_t callbackSequence = 0;
    int64_t rosTimeNs = 0;
    int64_t systemTimeNs = 0;
    int64_t steadyTimeNs = 0;
    OdomDiagnosticPose rawOdom;
    OdomDiagnosticPose odom;
    OdomDiagnosticPose odomToField;
    OdomDiagnosticPose field;
    OdomDiagnosticVelocityCommand command;
    uint64_t transformRevision = 0;
    double odomFactor = 1.0;
    double odomThetaAlignmentOffset = 0.0;
    double odomThetaAlignmentDistance = 0.0;
    double odomThetaAlignmentConcentration = 0.0;
    bool odomThetaAlignmentLocked = false;
    bool recoveryHoldActive = false;
    bool postGetupSettleActive = false;
    bool odomCalibrated = false;
    int controlState = 0;
    std::string gameState;
    std::string gameSubState;
    std::string gameSubStateType;
    std::string transformMode;
};

struct OdomDiagnosticTransformEvent
{
    int64_t rosTimeNs = 0;
    int64_t systemTimeNs = 0;
    int64_t steadyTimeNs = 0;
    std::string source;
    bool applied = false;
    uint64_t revisionBefore = 0;
    uint64_t revisionAfter = 0;
    OdomDiagnosticPose requestedFieldPose;
    OdomDiagnosticPose odom;
    OdomDiagnosticPose transformBefore;
    OdomDiagnosticPose transformAfter;
    OdomDiagnosticPose fieldBefore;
    OdomDiagnosticPose fieldAfter;
    double odomThetaAlignmentOffset = 0.0;
    double odomThetaAlignmentDistance = 0.0;
    double odomThetaAlignmentConcentration = 0.0;
    bool odomThetaAlignmentLocked = false;
    bool odomThetaAlignmentAnchorUsed = false;
    std::string odomThetaAlignmentAnchorSource;
    bool recoveryHoldActive = false;
    bool postGetupSettleActive = false;
};

struct OdomDiagnosticMetadata
{
    std::string gitDescribe;
    std::string buildDate;
    std::string buildTime;
    std::string executablePath;
    int playerId = 0;
    int teamId = 0;
    double sampleHz = 0.0;
    double flushIntervalMs = 1000.0;
    double odomFactor = 1.0;
    double odomThetaOffset = 0.0;
    bool odomThetaAutoAlign = false;
    double odomThetaAlignmentDistance = 0.0;
    double odomThetaAlignmentMinConcentration = 0.0;
};

class OdomDiagnosticLogger
{
public:
    OdomDiagnosticLogger(
        std::string filePath,
        const OdomDiagnosticMetadata &metadata);
    ~OdomDiagnosticLogger();

    OdomDiagnosticLogger(const OdomDiagnosticLogger &) = delete;
    OdomDiagnosticLogger &operator=(const OdomDiagnosticLogger &) = delete;

    bool enabled() const { return enabled_.load(); }
    const std::string &filePath() const { return filePath_; }
    const std::string &error() const { return error_; }

    bool shouldSample(std::chrono::steady_clock::time_point now);
    void enqueueSample(const OdomDiagnosticSample &sample);
    void enqueueTransformEvent(const OdomDiagnosticTransformEvent &event);

private:
    using Record = std::variant<
        OdomDiagnosticSample,
        OdomDiagnosticTransformEvent>;

    void enqueue(Record record);
    void writerLoop();
    void writeMetadata(const OdomDiagnosticMetadata &metadata);
    void writeSample(const OdomDiagnosticSample &sample);
    void writeTransformEvent(const OdomDiagnosticTransformEvent &event);

    static constexpr size_t kMaxQueuedRecords = 4096;

    std::string filePath_;
    std::string error_;
    std::ofstream stream_;
    std::atomic<bool> enabled_{false};
    std::atomic<uint64_t> droppedRecords_{0};
    double sampleHz_ = 0.0;
    std::chrono::milliseconds flushInterval_{1000};

    std::mutex sampleTimeMutex_;
    bool hasLastSampleTime_ = false;
    std::chrono::steady_clock::time_point lastSampleTime_;

    std::mutex queueMutex_;
    std::condition_variable queueCondition_;
    std::deque<Record> queue_;
    bool stopping_ = false;
    std::thread writerThread_;

    bool hasPreviousSample_ = false;
    OdomDiagnosticSample previousSample_;
};
