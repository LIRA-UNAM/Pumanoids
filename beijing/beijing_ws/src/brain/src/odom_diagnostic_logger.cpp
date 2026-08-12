#include "odom_diagnostic_logger.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <type_traits>
#include <utility>

namespace
{

double normalizeAngle(double angle)
{
    return std::remainder(angle, 2.0 * M_PI);
}

std::string jsonEscape(const std::string &value)
{
    std::ostringstream escaped;
    for (const unsigned char ch : value) {
        switch (ch) {
        case '"': escaped << "\\\""; break;
        case '\\': escaped << "\\\\"; break;
        case '\b': escaped << "\\b"; break;
        case '\f': escaped << "\\f"; break;
        case '\n': escaped << "\\n"; break;
        case '\r': escaped << "\\r"; break;
        case '\t': escaped << "\\t"; break;
        default:
            if (ch < 0x20) {
                escaped << "\\u"
                        << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(ch)
                        << std::dec << std::setfill(' ');
            } else {
                escaped << ch;
            }
        }
    }
    return escaped.str();
}

void writeNumber(std::ostream &stream, double value)
{
    if (std::isfinite(value)) {
        stream << value;
    } else {
        stream << "null";
    }
}

void writePose(std::ostream &stream, const OdomDiagnosticPose &pose)
{
    stream << "{\"x\":";
    writeNumber(stream, pose.x);
    stream << ",\"y\":";
    writeNumber(stream, pose.y);
    stream << ",\"theta\":";
    writeNumber(stream, pose.theta);
    stream << '}';
}

int64_t systemTimeNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

} // namespace

OdomDiagnosticLogger::OdomDiagnosticLogger(
    std::string filePath,
    const OdomDiagnosticMetadata &metadata)
    : filePath_(std::move(filePath)),
      sampleHz_(metadata.sampleHz),
      flushInterval_(static_cast<int64_t>(std::max(1.0, metadata.flushIntervalMs)))
{
    stream_.open(filePath_, std::ios::out | std::ios::trunc);
    if (!stream_.is_open()) {
        error_ = "cannot open file";
        return;
    }

    stream_ << std::setprecision(17);
    writeMetadata(metadata);
    stream_.flush();
    if (!stream_) {
        error_ = "failed to write metadata";
        stream_.close();
        return;
    }

    enabled_.store(true);
    writerThread_ = std::thread(&OdomDiagnosticLogger::writerLoop, this);
}

OdomDiagnosticLogger::~OdomDiagnosticLogger()
{
    if (!enabled_.load()) return;

    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        stopping_ = true;
    }
    queueCondition_.notify_one();
    if (writerThread_.joinable()) writerThread_.join();

    stream_ << "{\"type\":\"shutdown\",\"system_time_ns\":"
            << systemTimeNs()
            << ",\"dropped_records\":" << droppedRecords_.load()
            << "}\n";
    stream_.flush();
    stream_.close();
    enabled_.store(false);
}

bool OdomDiagnosticLogger::shouldSample(
    std::chrono::steady_clock::time_point now)
{
    if (!enabled_.load()) return false;
    if (!std::isfinite(sampleHz_) || sampleHz_ <= 0.0) return true;

    const std::chrono::duration<double> minimumInterval(1.0 / sampleHz_);
    std::lock_guard<std::mutex> lock(sampleTimeMutex_);
    if (!hasLastSampleTime_) {
        hasLastSampleTime_ = true;
        lastSampleTime_ = now;
        return true;
    }
    if (now - lastSampleTime_ < minimumInterval) return false;

    lastSampleTime_ = now;
    return true;
}

void OdomDiagnosticLogger::enqueueSample(
    const OdomDiagnosticSample &sample)
{
    enqueue(sample);
}

void OdomDiagnosticLogger::enqueueTransformEvent(
    const OdomDiagnosticTransformEvent &event)
{
    enqueue(event);
}

void OdomDiagnosticLogger::enqueue(Record record)
{
    if (!enabled_.load()) return;
    {
        std::lock_guard<std::mutex> lock(queueMutex_);
        if (stopping_ || queue_.size() >= kMaxQueuedRecords) {
            ++droppedRecords_;
            return;
        }
        queue_.push_back(std::move(record));
    }
    queueCondition_.notify_one();
}

void OdomDiagnosticLogger::writerLoop()
{
    auto lastFlush = std::chrono::steady_clock::now();
    while (true) {
        Record record;
        {
            std::unique_lock<std::mutex> lock(queueMutex_);
            queueCondition_.wait_for(lock, flushInterval_, [this]() {
                return stopping_ || !queue_.empty();
            });
            if (queue_.empty()) {
                if (stopping_) break;
                stream_.flush();
                lastFlush = std::chrono::steady_clock::now();
                continue;
            }
            record = std::move(queue_.front());
            queue_.pop_front();
        }

        std::visit([this](const auto &value) {
            using Value = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Value, OdomDiagnosticSample>) {
                writeSample(value);
            } else {
                writeTransformEvent(value);
            }
        }, record);

        const auto now = std::chrono::steady_clock::now();
        if (now - lastFlush >= flushInterval_) {
            stream_.flush();
            lastFlush = now;
        }
    }

    while (true) {
        Record record;
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            if (queue_.empty()) break;
            record = std::move(queue_.front());
            queue_.pop_front();
        }
        std::visit([this](const auto &value) {
            using Value = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Value, OdomDiagnosticSample>) {
                writeSample(value);
            } else {
                writeTransformEvent(value);
            }
        }, record);
    }
    stream_.flush();
}

void OdomDiagnosticLogger::writeMetadata(
    const OdomDiagnosticMetadata &metadata)
{
    stream_ << "{\"type\":\"metadata\",\"schema_version\":2"
            << ",\"semantics\":\"field_pose=odom_to_field*absolute_corrected_odom_pose; corrected_odom.theta=raw_odom.theta+odom_theta_alignment_offset; delta_field=R(odom_to_field.theta)*delta_odom when transform_revision is unchanged\""
            << ",\"source_timestamp\":\"unavailable_in_Odometer_msg; ros_time_ns is callback receipt time\""
            << ",\"git_describe\":\"" << jsonEscape(metadata.gitDescribe) << '"'
            << ",\"build_date\":\"" << jsonEscape(metadata.buildDate) << '"'
            << ",\"build_time\":\"" << jsonEscape(metadata.buildTime) << '"'
            << ",\"executable\":\"" << jsonEscape(metadata.executablePath) << '"'
            << ",\"player_id\":" << metadata.playerId
            << ",\"team_id\":" << metadata.teamId
            << ",\"sample_hz\":";
    writeNumber(stream_, metadata.sampleHz);
    stream_ << ",\"flush_interval_ms\":";
    writeNumber(stream_, metadata.flushIntervalMs);
    stream_ << ",\"odom_factor\":";
    writeNumber(stream_, metadata.odomFactor);
    stream_ << ",\"odom_theta_offset\":";
    writeNumber(stream_, metadata.odomThetaOffset);
    stream_ << ",\"odom_theta_auto_align\":"
            << (metadata.odomThetaAutoAlign ? "true" : "false")
            << ",\"odom_theta_alignment_distance\":";
    writeNumber(stream_, metadata.odomThetaAlignmentDistance);
    stream_ << ",\"odom_theta_alignment_min_concentration\":";
    writeNumber(stream_, metadata.odomThetaAlignmentMinConcentration);
    stream_ << "}\n";
}

void OdomDiagnosticLogger::writeSample(
    const OdomDiagnosticSample &sample)
{
    const double cosine = std::cos(sample.odomToField.theta);
    const double sine = std::sin(sample.odomToField.theta);
    const OdomDiagnosticPose recomputedField{
        sample.odomToField.x + sample.odom.x * cosine - sample.odom.y * sine,
        sample.odomToField.y + sample.odom.x * sine + sample.odom.y * cosine,
        normalizeAngle(sample.odom.theta + sample.odomToField.theta)};
    const OdomDiagnosticPose absoluteError{
        sample.field.x - recomputedField.x,
        sample.field.y - recomputedField.y,
        normalizeAngle(sample.field.theta - recomputedField.theta)};

    const double unavailable = std::numeric_limits<double>::quiet_NaN();
    double dtSeconds = unavailable;
    double callbackHz = unavailable;
    uint64_t callbacksSinceSample = 0;
    double deltaOdomX = unavailable;
    double deltaOdomY = unavailable;
    double deltaOdomTheta = unavailable;
    double deltaFieldX = unavailable;
    double deltaFieldY = unavailable;
    double deltaFieldTheta = unavailable;
    double expectedDeltaFieldX = unavailable;
    double expectedDeltaFieldY = unavailable;
    double deltaResidualX = unavailable;
    double deltaResidualY = unavailable;
    double deltaResidualTheta = unavailable;
    double estimatedBodyDeltaX = unavailable;
    double estimatedBodyDeltaY = unavailable;
    double odomMotionDirection = unavailable;
    double fieldMotionDirection = unavailable;
    double expectedFieldMotionDirection = unavailable;
    double commandExpectedOdomDirection = unavailable;
    double commandDirectionError = unavailable;
    double commandExpectedRawOdomDirection = unavailable;
    double rawOdomDirectionError = unavailable;
    bool fixedTransformDeltaValid = false;

    if (hasPreviousSample_) {
        dtSeconds = static_cast<double>(
            sample.steadyTimeNs - previousSample_.steadyTimeNs) / 1e9;
        callbacksSinceSample = sample.callbackSequence >= previousSample_.callbackSequence
            ? sample.callbackSequence - previousSample_.callbackSequence
            : 0;
        if (dtSeconds > 0.0) {
            callbackHz = static_cast<double>(callbacksSinceSample) / dtSeconds;
        }

        deltaOdomX = sample.odom.x - previousSample_.odom.x;
        deltaOdomY = sample.odom.y - previousSample_.odom.y;
        deltaOdomTheta = normalizeAngle(
            sample.odom.theta - previousSample_.odom.theta);
        deltaFieldX = sample.field.x - previousSample_.field.x;
        deltaFieldY = sample.field.y - previousSample_.field.y;
        deltaFieldTheta = normalizeAngle(
            sample.field.theta - previousSample_.field.theta);
        expectedDeltaFieldX = cosine * deltaOdomX - sine * deltaOdomY;
        expectedDeltaFieldY = sine * deltaOdomX + cosine * deltaOdomY;
        deltaResidualX = deltaFieldX - expectedDeltaFieldX;
        deltaResidualY = deltaFieldY - expectedDeltaFieldY;
        deltaResidualTheta = normalizeAngle(deltaFieldTheta - deltaOdomTheta);
        fixedTransformDeltaValid =
            sample.transformRevision == previousSample_.transformRevision;

        const double odomHeadingMid = normalizeAngle(
            previousSample_.odom.theta + 0.5 * deltaOdomTheta);
        const double headingCosine = std::cos(odomHeadingMid);
        const double headingSine = std::sin(odomHeadingMid);
        estimatedBodyDeltaX =
            headingCosine * deltaOdomX + headingSine * deltaOdomY;
        estimatedBodyDeltaY =
            -headingSine * deltaOdomX + headingCosine * deltaOdomY;

        if (std::hypot(deltaOdomX, deltaOdomY) > 1e-9) {
            odomMotionDirection = std::atan2(deltaOdomY, deltaOdomX);
            expectedFieldMotionDirection = normalizeAngle(
                odomMotionDirection + sample.odomToField.theta);
        }
        if (std::hypot(deltaFieldX, deltaFieldY) > 1e-9) {
            fieldMotionDirection = std::atan2(deltaFieldY, deltaFieldX);
        }
        if (std::hypot(sample.command.sentX, sample.command.sentY) > 1e-9) {
            commandExpectedOdomDirection = normalizeAngle(
                odomHeadingMid +
                std::atan2(sample.command.sentY, sample.command.sentX));
            const double rawDeltaTheta = normalizeAngle(
                sample.rawOdom.theta - previousSample_.rawOdom.theta);
            const double rawHeadingMid = normalizeAngle(
                previousSample_.rawOdom.theta + 0.5 * rawDeltaTheta);
            commandExpectedRawOdomDirection = normalizeAngle(
                rawHeadingMid +
                std::atan2(sample.command.sentY, sample.command.sentX));
            if (std::isfinite(odomMotionDirection)) {
                commandDirectionError = normalizeAngle(
                    odomMotionDirection - commandExpectedOdomDirection);
                rawOdomDirectionError = normalizeAngle(
                    odomMotionDirection - commandExpectedRawOdomDirection);
            }
        }
    }

    const double commandAgeMs = sample.command.rosTimeNs > 0
        ? static_cast<double>(sample.rosTimeNs - sample.command.rosTimeNs) / 1e6
        : unavailable;

    stream_ << "{\"type\":\"sample\""
            << ",\"seq\":" << sample.callbackSequence
            << ",\"ros_time_ns\":" << sample.rosTimeNs
            << ",\"system_time_ns\":" << sample.systemTimeNs
            << ",\"steady_time_ns\":" << sample.steadyTimeNs
            << ",\"callbacks_since_sample\":" << callbacksSinceSample
            << ",\"dt_s\":";
    writeNumber(stream_, dtSeconds);
    stream_ << ",\"callback_hz\":";
    writeNumber(stream_, callbackHz);
    stream_ << ",\"game_state\":\"" << jsonEscape(sample.gameState) << '"'
            << ",\"game_sub_state\":\"" << jsonEscape(sample.gameSubState) << '"'
            << ",\"game_sub_state_type\":\"" << jsonEscape(sample.gameSubStateType) << '"'
            << ",\"control_state\":" << sample.controlState
            << ",\"odom_calibrated\":" << (sample.odomCalibrated ? "true" : "false")
            << ",\"transform_mode\":\"" << jsonEscape(sample.transformMode) << '"'
            << ",\"recovery_hold_active\":" << (sample.recoveryHoldActive ? "true" : "false")
            << ",\"post_getup_settle_active\":" << (sample.postGetupSettleActive ? "true" : "false")
            << ",\"transform_revision\":" << sample.transformRevision
            << ",\"fixed_transform_delta_valid\":" << (fixedTransformDeltaValid ? "true" : "false")
            << ",\"odom_factor\":";
    writeNumber(stream_, sample.odomFactor);
    stream_ << ",\"odom_theta_alignment_offset\":";
    writeNumber(stream_, sample.odomThetaAlignmentOffset);
    stream_ << ",\"odom_theta_alignment_distance\":";
    writeNumber(stream_, sample.odomThetaAlignmentDistance);
    stream_ << ",\"odom_theta_alignment_concentration\":";
    writeNumber(stream_, sample.odomThetaAlignmentConcentration);
    stream_ << ",\"odom_theta_alignment_locked\":"
            << (sample.odomThetaAlignmentLocked ? "true" : "false");
    stream_ << ",\"raw_odom\":";
    writePose(stream_, sample.rawOdom);
    stream_ << ",\"odom\":";
    writePose(stream_, sample.odom);
    stream_ << ",\"odom_to_field\":";
    writePose(stream_, sample.odomToField);
    stream_ << ",\"field_actual\":";
    writePose(stream_, sample.field);
    stream_ << ",\"field_recomputed\":";
    writePose(stream_, recomputedField);
    stream_ << ",\"absolute_formula_error\":";
    writePose(stream_, absoluteError);
    stream_ << ",\"delta_odom\":{\"x\":";
    writeNumber(stream_, deltaOdomX);
    stream_ << ",\"y\":";
    writeNumber(stream_, deltaOdomY);
    stream_ << ",\"theta\":";
    writeNumber(stream_, deltaOdomTheta);
    stream_ << "},\"delta_field_actual\":{\"x\":";
    writeNumber(stream_, deltaFieldX);
    stream_ << ",\"y\":";
    writeNumber(stream_, deltaFieldY);
    stream_ << ",\"theta\":";
    writeNumber(stream_, deltaFieldTheta);
    stream_ << "},\"delta_field_expected\":{\"x\":";
    writeNumber(stream_, expectedDeltaFieldX);
    stream_ << ",\"y\":";
    writeNumber(stream_, expectedDeltaFieldY);
    stream_ << "},\"delta_formula_residual\":{\"x\":";
    writeNumber(stream_, deltaResidualX);
    stream_ << ",\"y\":";
    writeNumber(stream_, deltaResidualY);
    stream_ << ",\"theta\":";
    writeNumber(stream_, deltaResidualTheta);
    stream_ << "},\"estimated_body_delta\":{\"x\":";
    writeNumber(stream_, estimatedBodyDeltaX);
    stream_ << ",\"y\":";
    writeNumber(stream_, estimatedBodyDeltaY);
    stream_ << "},\"motion_direction\":{\"odom\":";
    writeNumber(stream_, odomMotionDirection);
    stream_ << ",\"field_actual\":";
    writeNumber(stream_, fieldMotionDirection);
    stream_ << ",\"field_expected\":";
    writeNumber(stream_, expectedFieldMotionDirection);
    stream_ << "},\"command\":{\"requested_vx\":";
    writeNumber(stream_, sample.command.requestedX);
    stream_ << ",\"requested_vy\":";
    writeNumber(stream_, sample.command.requestedY);
    stream_ << ",\"requested_vtheta\":";
    writeNumber(stream_, sample.command.requestedTheta);
    stream_ << ",\"sent_vx\":";
    writeNumber(stream_, sample.command.sentX);
    stream_ << ",\"sent_vy\":";
    writeNumber(stream_, sample.command.sentY);
    stream_ << ",\"sent_vtheta\":";
    writeNumber(stream_, sample.command.sentTheta);
    stream_ << ",\"ros_time_ns\":" << sample.command.rosTimeNs
            << ",\"age_ms\":";
    writeNumber(stream_, commandAgeMs);
    stream_ << ",\"expected_odom_direction\":";
    writeNumber(stream_, commandExpectedOdomDirection);
    stream_ << ",\"odom_direction_error\":";
    writeNumber(stream_, commandDirectionError);
    stream_ << ",\"expected_raw_odom_direction\":";
    writeNumber(stream_, commandExpectedRawOdomDirection);
    stream_ << ",\"raw_odom_direction_error\":";
    writeNumber(stream_, rawOdomDirectionError);
    stream_ << "}}\n";

    previousSample_ = sample;
    hasPreviousSample_ = true;
}

void OdomDiagnosticLogger::writeTransformEvent(
    const OdomDiagnosticTransformEvent &event)
{
    stream_ << "{\"type\":\"transform_event\""
            << ",\"ros_time_ns\":" << event.rosTimeNs
            << ",\"system_time_ns\":" << event.systemTimeNs
            << ",\"steady_time_ns\":" << event.steadyTimeNs
            << ",\"source\":\"" << jsonEscape(event.source) << '"'
            << ",\"applied\":" << (event.applied ? "true" : "false")
            << ",\"revision_before\":" << event.revisionBefore
            << ",\"revision_after\":" << event.revisionAfter
            << ",\"recovery_hold_active\":" << (event.recoveryHoldActive ? "true" : "false")
            << ",\"post_getup_settle_active\":" << (event.postGetupSettleActive ? "true" : "false")
            << ",\"odom_theta_alignment_locked\":"
            << (event.odomThetaAlignmentLocked ? "true" : "false")
            << ",\"odom_theta_alignment_anchor_used\":"
            << (event.odomThetaAlignmentAnchorUsed ? "true" : "false")
            << ",\"odom_theta_alignment_anchor_source\":\""
            << jsonEscape(event.odomThetaAlignmentAnchorSource) << '"'
            << ",\"odom_theta_alignment_offset\":";
    writeNumber(stream_, event.odomThetaAlignmentOffset);
    stream_ << ",\"odom_theta_alignment_distance\":";
    writeNumber(stream_, event.odomThetaAlignmentDistance);
    stream_ << ",\"odom_theta_alignment_concentration\":";
    writeNumber(stream_, event.odomThetaAlignmentConcentration);
    stream_ << ",\"requested_field_pose\":";
    writePose(stream_, event.requestedFieldPose);
    stream_ << ",\"odom\":";
    writePose(stream_, event.odom);
    stream_ << ",\"transform_before\":";
    writePose(stream_, event.transformBefore);
    stream_ << ",\"transform_after\":";
    writePose(stream_, event.transformAfter);
    stream_ << ",\"field_before\":";
    writePose(stream_, event.fieldBefore);
    stream_ << ",\"field_after\":";
    writePose(stream_, event.fieldAfter);
    stream_ << "}\n";
}
