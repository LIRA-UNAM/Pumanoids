#include "obstacle_avoidance_logger.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

void expect(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    namespace fs = std::filesystem;
    const auto unique = std::chrono::steady_clock::now()
        .time_since_epoch().count();
    const fs::path directory = fs::temp_directory_path() /
        ("obstacle_avoidance_logger_test_" + std::to_string(unique));
    fs::create_directories(directory);
    const fs::path logPath = directory / "obstacle_avoidance.log";

    {
        ObstacleAvoidanceLogger logger(logPath.string(), 0.0, 700, 2);
        expect(logger.enabled(), "logger did not open its output file");
        ObstacleAvoidanceLogRecord record;
        record.source = "move2";
        record.reason = "blocked_stop";
        record.depthObstacleCount = 64;
        record.ballObstacleCount = 1;
        record.depthGridMaxOccupancy = 42;
        record.depthGridConfirmedCells = 3;
        record.occupancyThreshold = 20;
        record.robotMinimumDistance = 0.20;
        record.avoidanceReverseSpeed = 0.35;
        record.buildVersion = "test-build";
        record.obstacles.push_back(
            {"depth", 0.5, 0.2, 0.8, 600.0, 75.0, 2, 0, false});
        for (int index = 0; index < 8; ++index) logger.log(record);
    }

    expect(fs::exists(logPath), "active log file was not created");
    expect(fs::exists(logPath.string() + ".1"),
           "size-limited log file was not rotated");
    std::ifstream stream(logPath.string() + ".1");
    const std::string text{
        std::istreambuf_iterator<char>(stream),
        std::istreambuf_iterator<char>()};
    expect(text.find("\"source\":\"move2\"") != std::string::npos,
           "avoidance source was not serialized");
    expect(text.find("\"depth_count\":64") != std::string::npos,
           "obstacle count was not serialized");
    expect(text.find("\"ball_count\":1") != std::string::npos,
           "ball obstacle count was not serialized");
    expect(text.find("\"depth_grid_max_occupancy\":42") != std::string::npos,
           "depth occupancy diagnostic was not serialized");
    expect(text.find("\"build_version\":\"test-build\"") != std::string::npos,
           "build version was not serialized");
    expect(text.find("\"blocked_duration_ms\"") != std::string::npos,
           "blocked duration was not serialized");
    expect(text.find("\"avoidance_reverse_speed\":0.35") !=
               std::string::npos,
           "reverse avoidance speed was not serialized");
    expect(text.find("\"robot_minimum_distance\":0.2") !=
               std::string::npos,
           "minimum robot distance was not serialized");
    expect(text.find("],\"depth_sample_step\"") != std::string::npos,
           "obstacle array was not closed before metadata fields");

    std::error_code cleanupError;
    fs::remove_all(directory, cleanupError);
    return 0;
}
