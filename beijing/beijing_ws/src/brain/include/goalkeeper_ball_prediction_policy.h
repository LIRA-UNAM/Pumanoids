#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace goalkeeper_prediction {

struct Observation
{
    double timeSec = 0.0;
    double x = 0.0;
    double y = 0.0;
};

struct Config
{
    std::size_t minSamples = 5;
    double minSpanSec = 0.25;
    double minSpeed = 0.45;
    double minTowardGoalSpeed = 0.35;
    double minRSquared = 0.90;
    double maxResidual = 0.20;
    double recencyWeight = 1.0;
    double deceleration = 0.40;
    double stepSec = 0.10;
    std::size_t stepCount = 30;
    double blockLineX = 0.0;
    double goalHalfWidth = 1.3;
    double goalMargin = 0.15;
    double minTimeToBlock = 0.08;
    double maxTimeToBlock = 2.50;
};

struct Result
{
    bool fitComputed = false;
    bool valid = false;
    bool movingTowardOwnGoal = false;
    bool willReachBlockLine = false;
    bool threatensGoal = false;
    std::string reason = "insufficient_samples";
    double velocityX = 0.0;
    double velocityY = 0.0;
    double speed = 0.0;
    double rSquared = 0.0;
    double rSquaredX = 0.0;
    double rSquaredY = 0.0;
    double residualRms = std::numeric_limits<double>::infinity();
    double sampleSpanSec = 0.0;
    double timeToBlock = std::numeric_limits<double>::infinity();
    double interceptX = 0.0;
    double interceptY = 0.0;
    std::vector<Observation> trajectory;
};

namespace detail {

struct Fit
{
    double slope = 0.0;
    double intercept = 0.0;
    double rSquared = 0.0;
    double residualSquared = 0.0;
};

inline Fit fitLine(const std::vector<double> &time,
                   const std::vector<double> &value,
                   double recencyWeight)
{
    Fit result;
    if (time.size() < 2 || time.size() != value.size()) return result;

    double meanTime = 0.0;
    double meanValue = 0.0;
    double weightSum = 0.0;
    const double safeRecencyWeight = std::max(1.0, recencyWeight);
    for (std::size_t i = 0; i < time.size(); ++i) {
        const double progress = time.size() <= 1
            ? 1.0
            : static_cast<double>(i) /
                static_cast<double>(time.size() - 1);
        const double weight = 1.0 +
            (safeRecencyWeight - 1.0) * progress;
        meanTime += weight * time[i];
        meanValue += weight * value[i];
        weightSum += weight;
    }
    meanTime /= weightSum;
    meanValue /= weightSum;

    double covariance = 0.0;
    double varianceTime = 0.0;
    double totalSquared = 0.0;
    for (std::size_t i = 0; i < time.size(); ++i) {
        const double progress = time.size() <= 1
            ? 1.0
            : static_cast<double>(i) /
                static_cast<double>(time.size() - 1);
        const double weight = 1.0 +
            (safeRecencyWeight - 1.0) * progress;
        const double dt = time[i] - meanTime;
        const double dv = value[i] - meanValue;
        covariance += weight * dt * dv;
        varianceTime += weight * dt * dt;
        totalSquared += weight * dv * dv;
    }
    if (varianceTime <= 1e-12) return result;

    result.slope = covariance / varianceTime;
    result.intercept = meanValue - result.slope * meanTime;
    for (std::size_t i = 0; i < time.size(); ++i) {
        const double progress = time.size() <= 1
            ? 1.0
            : static_cast<double>(i) /
                static_cast<double>(time.size() - 1);
        const double weight = 1.0 +
            (safeRecencyWeight - 1.0) * progress;
        const double error = value[i] -
            (result.slope * time[i] + result.intercept);
        result.residualSquared += weight * error * error;
    }
    result.rSquared = totalSquared <= 1e-12
        ? (result.residualSquared <= 1e-12 ? 1.0 : 0.0)
        : std::clamp(1.0 - result.residualSquared / totalSquared, 0.0, 1.0);
    return result;
}

inline double travelDistance(double speed, double deceleration, double timeSec)
{
    if (timeSec <= 0.0 || speed <= 0.0) return 0.0;
    if (deceleration <= 1e-9) return speed * timeSec;
    const double stopTime = speed / deceleration;
    const double boundedTime = std::min(timeSec, stopTime);
    return speed * boundedTime -
        0.5 * deceleration * boundedTime * boundedTime;
}

inline double timeForDistance(double speed, double deceleration,
                              double distance)
{
    if (distance < 0.0 || speed <= 1e-9) {
        return std::numeric_limits<double>::infinity();
    }
    if (deceleration <= 1e-9) return distance / speed;
    const double discriminant = speed * speed -
        2.0 * deceleration * distance;
    if (discriminant < 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    // This equivalent quadratic root is stable when deceleration is small.
    return 2.0 * distance /
        (speed + std::sqrt(std::max(0.0, discriminant)));
}

} // namespace detail

inline Result predict(const std::vector<Observation> &observations,
                      const Config &config)
{
    Result result;
    if (observations.size() < 2) {
        return result;
    }

    const double firstTime = observations.front().timeSec;
    const double lastTime = observations.back().timeSec;
    if (!std::isfinite(firstTime) || !std::isfinite(lastTime)) {
        result.reason = "invalid_observation";
        return result;
    }
    result.sampleSpanSec = std::max(0.0, lastTime - firstTime);

    std::vector<double> time;
    std::vector<double> x;
    std::vector<double> y;
    time.reserve(observations.size());
    x.reserve(observations.size());
    y.reserve(observations.size());
    for (const auto &observation : observations) {
        if (!std::isfinite(observation.timeSec) ||
            !std::isfinite(observation.x) || !std::isfinite(observation.y)) {
            result.reason = "invalid_observation";
            return result;
        }
        time.push_back(observation.timeSec - lastTime);
        x.push_back(observation.x);
        y.push_back(observation.y);
    }

    const detail::Fit fitX = detail::fitLine(
        time, x, config.recencyWeight);
    const detail::Fit fitY = detail::fitLine(
        time, y, config.recencyWeight);
    result.velocityX = fitX.slope;
    result.velocityY = fitY.slope;
    result.speed = std::hypot(result.velocityX, result.velocityY);
    result.rSquaredX = fitX.rSquared;
    result.rSquaredY = fitY.rSquared;
    result.fitComputed = true;
    // A shot toward the own goal must have a changing x coordinate. Use the
    // x fit for the acceptance gate and report the weaker fit for diagnosis.
    result.rSquared = std::min(fitX.rSquared, fitY.rSquared);
    const double weightSum = observations.size() <= 1
        ? 1.0
        : static_cast<double>(observations.size()) *
            (1.0 + std::max(1.0, config.recencyWeight)) / 2.0;
    result.residualRms = std::sqrt(
        (fitX.residualSquared + fitY.residualSquared) / weightSum);

    if (observations.size() < config.minSamples) {
        result.reason = "insufficient_samples";
        return result;
    }
    if (result.sampleSpanSec < config.minSpanSec) {
        result.reason = "insufficient_span";
        return result;
    }

    if (result.speed < config.minSpeed) {
        result.reason = "speed_below_minimum";
        return result;
    }
    if (fitX.rSquared < config.minRSquared) {
        result.reason = "r_squared_below_minimum";
        return result;
    }
    if (result.residualRms > config.maxResidual) {
        result.reason = "residual_above_maximum";
        return result;
    }
    result.valid = true;

    result.movingTowardOwnGoal =
        result.velocityX <= -std::abs(config.minTowardGoalSpeed);
    if (!result.movingTowardOwnGoal) {
        result.reason = "not_toward_own_goal";
        return result;
    }

    const Observation &latest = observations.back();
    const double unitX = result.velocityX / result.speed;
    const double unitY = result.velocityY / result.speed;
    if (unitX >= -1e-9 || latest.x <= config.blockLineX) {
        result.reason = "already_past_block_line";
        return result;
    }

    const double travelToLine = (config.blockLineX - latest.x) / unitX;
    result.timeToBlock = detail::timeForDistance(
        result.speed, std::max(0.0, config.deceleration), travelToLine);
    if (!std::isfinite(result.timeToBlock)) {
        result.reason = "stops_before_block_line";
        return result;
    }

    result.willReachBlockLine = true;
    result.interceptX = config.blockLineX;
    result.interceptY = latest.y + unitY * travelToLine;
    result.threatensGoal =
        result.timeToBlock >= config.minTimeToBlock &&
        result.timeToBlock <= config.maxTimeToBlock &&
        std::abs(result.interceptY) <=
            config.goalHalfWidth + std::max(0.0, config.goalMargin);
    if (result.timeToBlock < config.minTimeToBlock ||
        result.timeToBlock > config.maxTimeToBlock) {
        result.reason = "time_outside_window";
    } else if (std::abs(result.interceptY) >
               config.goalHalfWidth + std::max(0.0, config.goalMargin)) {
        result.reason = "outside_goal";
    } else {
        result.reason = "threat_detected";
    }

    const double stepSec = std::max(0.01, config.stepSec);
    for (std::size_t i = 1; i <= config.stepCount; ++i) {
        const double futureTime = stepSec * static_cast<double>(i);
        const double distance = detail::travelDistance(
            result.speed, std::max(0.0, config.deceleration), futureTime);
        result.trajectory.push_back({
            lastTime + futureTime,
            latest.x + unitX * distance,
            latest.y + unitY * distance,
        });
        if (config.deceleration > 1e-9 &&
            futureTime >= result.speed / config.deceleration) break;
    }
    return result;
}

} // namespace goalkeeper_prediction
