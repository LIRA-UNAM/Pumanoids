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

    std::vector<Observation> wide;
    for (int i = 0; i < 7; ++i) {
        const double t = 0.1 * i;
        wide.push_back({t, -1.0 - 2.0 * t, 2.2});
    }
    const auto missesGoal = goalkeeper_prediction::predict(wide, config);
    assert(missesGoal.valid);
    assert(missesGoal.willReachBlockLine);
    assert(!missesGoal.threatensGoal);

    config.deceleration = 4.0;
    const auto stopsEarly = goalkeeper_prediction::predict(incoming, config);
    assert(stopsEarly.valid);
    assert(!stopsEarly.willReachBlockLine);
    assert(!stopsEarly.threatensGoal);

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

    std::cout << "goalkeeper_ball_prediction_policy_test passed\n";
    return 0;
}
