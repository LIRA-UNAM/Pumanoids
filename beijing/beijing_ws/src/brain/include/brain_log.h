#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <rerun.hpp>
#include "types.h"


class Brain; // Forward declaration

using namespace std;

/**
 * Centralizes operations related to Rerun logging.
 * Reusable log generation can be encapsulated in this class.
 */
class BrainLog
{
public:
    /**
     * The constructor only initializes the object and sets enabled to false.
     */
    BrainLog(Brain *argBrain);

    // Write static data, such as field markings, to the Rerun log.
    void prepare();

    // Write static content, such as field markings, to the Rerun log.
    void logStatics();

    // Set the log time to the current time.
    void setTimeNow();

    // Set the log time explicitly.
    void setTimeSeconds(double seconds);

    // Whether Rerun logging is active, allowing callers to skip expensive visualization construction.
    bool enabled() const { return enable_log_tcp || enable_log_file; }

    // Rate-limit high-frequency logs by call site. max_hz <= 0 disables throttling.
    bool shouldLog(std::string_view throttle_key, double max_hz);

    template <typename... Ts>
    inline void logThrottled(
        std::string_view throttle_key,
        double max_hz,
        std::string_view entity_path,
        const Ts &...archetypes_or_collections)
    {
        if (shouldLog(throttle_key, max_hz)) {
            log(entity_path, archetypes_or_collections...);
        }
    }

    // Expose an interface compatible with rerun::RecordingStream.
    template <typename... Ts>
    inline void log(string_view entity_path, const Ts &...archetypes_or_collections) const
    {
        if (!enabled()) {
            return;
        }

        if (enable_log_tcp) {
            log_tcp.log(entity_path, archetypes_or_collections...);
        }

        if (enable_log_file) {
            log_file.log(entity_path, archetypes_or_collections...);
        }
    }

    // Annotate an image log with a colored border outside the padding and a text label below it.
    void logToScreen(string logPath, string text, u_int32_t color, double padding = 0.0);

    void logRobot(string logPath, Pose2D pose, u_int32_t color, string label = "", bool draw2mCircle = false);

    void logBall(string logPath, Point pos, u_int32_t color, bool detected, bool known);

    rerun::LineStrip2D circle (float x, float y, float r, int nSeg = 36);
    rerun::LineStrip2D crosshair(float x, float y, float r);

    // Switch the log stream to a new file to prevent files from becoming too large to read.
    void updateLogFilePath();

private:
    Brain *brain;
    bool enable_log_tcp = false;
    bool enable_log_file = false;
    rerun::RecordingStream log_tcp;
    rerun::RecordingStream log_file;
    std::mutex rate_limit_mutex;
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_log_times;
};
