#include "fall_recovery_policy.h"

#include <iostream>
#include <limits>

namespace policy = fall_recovery_policy;

int main()
{
    int failures = 0;
    const auto expect = [&](bool condition, const char *message) {
        if (!condition) {
            std::cerr << message << '\n';
            ++failures;
        }
    };

    expect(!policy::shouldEnterLocalizationPreHold(0.42, 0.10),
           "normal body tilt entered localization pre-hold");
    expect(policy::shouldEnterLocalizationPreHold(0.10, 0.49),
           "early fall did not enter localization pre-hold");
    expect(policy::isLocalizationPreHoldReleaseTilt(0.20, 0.29),
           "upright pose did not satisfy release tilt");
    expect(!policy::isLocalizationPreHoldReleaseTilt(
               std::numeric_limits<double>::quiet_NaN(), 0.0),
           "invalid attitude released localization pre-hold");

    const double heldHeading = policy::fieldHeadingWithHeldTranslation(
        -0.95565, -1.67992, -1.19492);
    expect(std::abs(heldHeading - (-0.47065)) < 1e-9,
           "fall hold discarded a trustworthy odometer heading delta");
    const double wrappedHeading = policy::fieldHeadingWithHeldTranslation(
        3.10, 3.05, -3.05);
    const double expectedWrapped = std::atan2(
        std::sin(3.10 + std::atan2(std::sin(-6.10), std::cos(-6.10))),
        std::cos(3.10 + std::atan2(std::sin(-6.10), std::cos(-6.10))));
    expect(std::abs(wrappedHeading - expectedWrapped) < 1e-12,
           "fall hold heading propagation failed across the pi wrap");

    expect(policy::isStationaryUprightSample(
               0.20, 0.20, 0.02, 0.01, 0.01, 0.10),
           "valid stationary upright sample was rejected");
    expect(!policy::isStationaryUprightSample(
               0.31, 0.0, 0.0, 0.0, 0.0, 0.10),
           "excessive tilt was accepted as upright");
    expect(!policy::isStationaryUprightSample(
               0.0,
               0.0,
               std::numeric_limits<double>::quiet_NaN(),
               0.0,
               0.0,
               0.10),
           "invalid gyro was accepted as stationary");
    expect(!policy::isStationaryUprightSample(
               0.0, 0.0, 0.08, 0.08, 0.0, 0.10),
           "moving body was accepted as stationary");

    std::int64_t stableSinceUs = 0;
    expect(!policy::updateUprightRecoveryConfirmation(
               false, true, 1000000, stableSinceUs) && stableSinceUs == 0,
           "moving sample armed recovery confirmation");
    expect(!policy::updateUprightRecoveryConfirmation(
               false, false, 1100000, stableSinceUs),
           "first upright sample confirmed recovery");
    expect(!policy::updateUprightRecoveryConfirmation(
               false, false, 2050000, stableSinceUs),
           "sub-second interval confirmed recovery");
    expect(policy::updateUprightRecoveryConfirmation(
               false, false, 2100000, stableSinceUs),
           "one-second interval did not confirm recovery");
    expect(!policy::updateUprightRecoveryConfirmation(
               true, false, 2200000, stableSinceUs) && stableSinceUs == 0,
           "unsafe posture did not revoke recovery confirmation");

    expect(policy::areHeadingCandidatesConsistent(3.10, -3.10, 0.10),
           "heading consistency did not handle the pi wrap");
    expect(!policy::areHeadingCandidatesConsistent(0.0, 0.4, 0.25),
           "inconsistent recovery headings were accepted");
    expect(policy::isSafeHeadingRealignCandidate(
               -2.0, -1.8, 0.2, -1.4, 0.5, 2.5),
           "safe same-half heading candidate was rejected");
    expect(!policy::isSafeHeadingRealignCandidate(
               -2.0, -1.8, 0.2, 1.4, 0.5, 2.5),
           "cross-half heading candidate was accepted");
    expect(!policy::isSafeHeadingRealignCandidate(
               -2.0, -1.8, 0.2, -4.5, 0.5, 2.5),
           "far heading candidate was accepted");

    policy::PostGetupOdomSettleInput odomSettle;
    odomSettle.minimumDwellMs = 3500.0;
    odomSettle.maximumHoldMs = 7500.0;
    odomSettle.odomWindowReady = true;
    odomSettle.odomFinite = true;
    odomSettle.imuUpright = true;
    odomSettle.gyroQuiet = true;
    odomSettle.odomXRangeM = 0.003;
    odomSettle.odomYRangeM = 0.005;
    odomSettle.odomThetaRangeRad = 0.0028;
    odomSettle.maximumXyRangeM = 0.015;
    odomSettle.maximumThetaRangeRad = 0.0131;

    odomSettle.elapsedMs = 1200.0;
    auto odomDecision = policy::evaluatePostGetupOdomSettling(odomSettle);
    expect(odomDecision.active && !odomDecision.stable &&
               std::string(odomDecision.reason) == "minimum_dwell",
           "quiet false plateau bypassed the mandatory post-getup dwell");

    odomSettle.elapsedMs = 4000.0;
    odomSettle.odomWindowReady = false;
    odomDecision = policy::evaluatePostGetupOdomSettling(odomSettle);
    expect(odomDecision.active &&
               std::string(odomDecision.reason) == "odom_window_incomplete",
           "incomplete odometry window released the post-getup hold");

    odomSettle.odomWindowReady = true;
    odomSettle.odomThetaRangeRad = 0.04;
    odomDecision = policy::evaluatePostGetupOdomSettling(odomSettle);
    expect(odomDecision.active &&
               std::string(odomDecision.reason) == "odom_heading_unstable",
           "unstable post-getup heading released localization");

    odomSettle.odomThetaRangeRad = 0.0028;
    odomSettle.odomYRangeM = 0.025;
    odomDecision = policy::evaluatePostGetupOdomSettling(odomSettle);
    expect(odomDecision.active &&
               std::string(odomDecision.reason) == "odom_translation_unstable",
           "unstable post-getup translation released localization");

    odomSettle.odomYRangeM = 0.005;
    odomDecision = policy::evaluatePostGetupOdomSettling(odomSettle);
    expect(!odomDecision.active && odomDecision.stable &&
               !odomDecision.timedOut,
           "complete quiet odometry window did not release localization");

    odomSettle.elapsedMs = 7500.0;
    odomSettle.gyroQuiet = false;
    odomDecision = policy::evaluatePostGetupOdomSettling(odomSettle);
    expect(odomDecision.active && !odomDecision.timedOut,
           "maximum timeout released localization while the body was moving");

    odomSettle.gyroQuiet = true;
    odomSettle.odomThetaRangeRad = 1.0;
    odomDecision = policy::evaluatePostGetupOdomSettling(odomSettle);
    expect(!odomDecision.active && odomDecision.timedOut,
           "post-getup odometry hold ignored its bounded timeout");

    if (failures == 0) {
        std::cout << "fall recovery policy tests passed\n";
    }
    return failures == 0 ? 0 : 1;
}
