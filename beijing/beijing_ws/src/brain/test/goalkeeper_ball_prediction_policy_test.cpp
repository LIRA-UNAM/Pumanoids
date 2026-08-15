#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "goalkeeper_ball_prediction_policy.h"

using goalkeeper_prediction::Config;
using goalkeeper_prediction::Observation;

int main()
{
    Config config;
    config.minSamples = 5;
    config.minSpanSec = 0.3;
    config.blockLineX = -4.2;
    config.goalHalfWidth = 1.3;
    config.deceleration = 0.0;

    std::vector<Observation> incoming;
    for (int i = 0; i < 7; ++i) {
        const double t = 0.1 * i;
        incoming.push_back({t, -1.0 - 2.0 * t, 0.3 + 0.2 * t});
    }
    const auto shot = goalkeeper_prediction::predict(incoming, config);
    assert(shot.valid);
    assert(shot.movingTowardOwnGoal);
    assert(shot.willReachBlockLine);
    assert(shot.threatensGoal);
    assert(shot.reason == "threat_detected");
    assert(shot.fitComputed);
    assert(std::abs(shot.sampleSpanSec - 0.6) < 1e-9);
    assert(shot.rSquaredX > 0.999);
    assert(std::abs(shot.velocityX + 2.0) < 1e-6);
    assert(std::abs(shot.velocityY - 0.2) < 1e-6);

    std::vector<Observation> outgoing;
    for (int i = 0; i < 7; ++i) {
        const double t = 0.1 * i;
        outgoing.push_back({t, -3.0 + 1.0 * t, 0.0});
    }
    const auto away = goalkeeper_prediction::predict(outgoing, config);
    assert(away.valid);
    assert(!away.movingTowardOwnGoal);
    assert(!away.threatensGoal);
    assert(away.reason == "not_toward_own_goal");

    std::vector<Observation> wide;
    for (int i = 0; i < 7; ++i) {
        const double t = 0.1 * i;
        wide.push_back({t, -1.0 - 2.0 * t, 2.2});
    }
    const auto missesGoal = goalkeeper_prediction::predict(wide, config);
    assert(missesGoal.valid);
    assert(missesGoal.willReachBlockLine);
    assert(!missesGoal.threatensGoal);
    assert(missesGoal.reason == "outside_goal");

    config.deceleration = 4.0;
    const auto stopsEarly = goalkeeper_prediction::predict(incoming, config);
    assert(stopsEarly.valid);
    assert(!stopsEarly.willReachBlockLine);
    assert(!stopsEarly.threatensGoal);
    assert(stopsEarly.reason == "stops_before_block_line");

    config.deceleration = 0.0;
    config.recencyWeight = 4.0;
    auto accelerated = incoming;
    accelerated[0].x += 0.08;
    accelerated[1].x += 0.05;
    const auto weighted = goalkeeper_prediction::predict(
        accelerated, config);
    assert(weighted.valid);
    assert(weighted.movingTowardOwnGoal);
    assert(weighted.threatensGoal);

    auto noisy = incoming;
    noisy[3].x += 1.0;
    const auto rejected = goalkeeper_prediction::predict(noisy, config);
    assert(!rejected.valid);
    assert(rejected.reason == "r_squared_below_minimum" ||
           rejected.reason == "residual_above_maximum");

    Config fastConfig = config;
    fastConfig.minSamples = 5;
    fastConfig.minSpanSec = 0.25;
    std::vector<Observation> shortSpan(incoming.begin(), incoming.begin() + 5);
    for (std::size_t i = 0; i < shortSpan.size(); ++i) {
        shortSpan[i].timeSec = 0.02 * static_cast<double>(i);
    }
    const auto partial = goalkeeper_prediction::predict(shortSpan, fastConfig);
    assert(!partial.valid);
    assert(partial.fitComputed);
    assert(partial.reason == "insufficient_span");
    assert(partial.speed > 0.0);

    Config implausibleConfig = config;
    implausibleConfig.maxSpeed = 8.0;
    std::vector<Observation> implausible;
    for (int i = 0; i < 7; ++i) {
        const double t = 0.1 * i;
        implausible.push_back({t, -1.0 - 12.0 * t, 0.0});
    }
    const auto impossibleShot = goalkeeper_prediction::predict(
        implausible, implausibleConfig);
    assert(!impossibleShot.valid);
    assert(impossibleShot.reason == "speed_above_maximum");

    std::cout << "goalkeeper_ball_prediction_policy_test passed\n";
    return 0;
}
