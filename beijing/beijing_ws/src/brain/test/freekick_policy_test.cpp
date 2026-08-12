#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "../include/freekick_policy.h"

namespace policy = freekick_policy;

namespace {

void require(bool condition, const char *message)
{
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void checkGeometry(const policy::Field &field)
{
    const double xLimit = field.length / 2.0 - 0.35;
    const double yLimit = field.width / 2.0 - 0.35;
    for (const policy::Point &ball : {
             policy::Point{0.0, 0.0},
             policy::Point{xLimit, yLimit},
             policy::Point{-xLimit, -yLimit},
         }) {
        const policy::Point receiver{
            std::clamp(ball.x + 2.0, -xLimit, xLimit),
            std::clamp(ball.y - 1.0, -yLimit, yLimit),
        };
        const policy::Point target = policy::chooseIndirectPassTarget(
            ball, receiver, {}, field);
        require(std::isfinite(target.x) && std::isfinite(target.y),
                "pass target is finite");
        require(std::fabs(target.x) <= xLimit + 1e-9,
                "pass target remains inside field length");
        require(std::fabs(target.y) <= yLimit + 1e-9,
                "pass target remains inside field width");
    }
}

void checkOpponentAwareSelection()
{
    const policy::Field field{14.16, 9.22};
    const policy::Point ball{-1.0, 0.0};
    const policy::Point receiver{2.0, 0.0};
    const policy::Point directTarget =
        policy::chooseIndirectPassTarget(ball, receiver, {}, field);
    const std::vector<policy::Point> opponents{
        {0.5, 0.0},
        {directTarget.x, directTarget.y},
    };
    const policy::Point protectedTarget =
        policy::chooseIndirectPassTarget(ball, receiver, opponents, field);
    require(
        policy::nearestOpponentDistance(protectedTarget, opponents) >
            policy::nearestOpponentDistance(directTarget, opponents) + 0.2 ||
        policy::passLaneClearance(ball, protectedTarget, opponents) >
            policy::passLaneClearance(ball, directTarget, opponents) + 0.2,
        "opponents cause a meaningfully safer pass target");
}

void checkLifecycle()
{
    using Phase = policy::Phase;
    using Type = policy::Type;
    require(policy::actorForPhase(Type::Indirect, Phase::Released, 2, 4) == 2,
            "kicker owns the first touch");
    require(policy::kickStarted(Type::Indirect, Phase::Released) ==
                Phase::FirstTouchKicking,
            "first kick starts only from release");
    require(policy::kickCompleted(Type::Indirect, Phase::FirstTouchKicking) ==
                Phase::AwaitingSecondTouch,
            "indirect first touch waits for a receiver");
    require(policy::actorForPhase(
                Type::Indirect, Phase::AwaitingSecondTouch, 2, 4) == 4,
            "receiver owns the second touch");
    require(policy::kickStarted(
                Type::Indirect, Phase::AwaitingSecondTouch) ==
                Phase::SecondTouchKicking,
            "receiver starts the second touch");
    require(policy::kickCompleted(
                Type::Indirect, Phase::SecondTouchKicking) == Phase::Complete,
            "second touch completes an indirect free kick");
    require(policy::kickCompleted(Type::Direct, Phase::FirstTouchKicking) ==
                Phase::Complete,
            "one touch completes a direct free kick");
    require(policy::actorForPhase(Type::Indirect, Phase::Released, 2, 0) == 0,
            "an indirect kick without a receiver has no actor");
    require(!policy::hasValidReceiver(Type::Indirect, 2, 0, 4),
            "indirect plans require a distinct receiver");
    require(policy::hasValidReceiver(Type::Indirect, 2, 4, 4),
            "indirect plans accept a valid receiver");
    require(policy::visualKickContextAllowed(false, false, true),
            "normal play allows an eligible leader to use visual kick");
    require(policy::visualKickContextAllowed(true, true, false),
            "the planned free-kick actor may use visual kick");
    require(!policy::visualKickContextAllowed(true, false, true),
            "a free-kick non-actor may not use visual kick");
    require(!policy::visualKickContextAllowed(false, false, false),
            "an ineligible normal-play robot may not use visual kick");
    require(!policy::executionExpired(
                Phase::Released, 10000.0, 10000.0),
            "the free-kick execution remains active at its exact deadline");
    require(policy::executionExpired(
                Phase::FirstTouchKicking, 10000.1, 10000.0),
            "an active free-kick execution expires after its deadline");
    require(!policy::executionExpired(
                Phase::Preparing, 20000.0, 10000.0),
            "the execution timeout does not apply before referee release");
    require(!policy::executionExpired(
                Phase::Complete, 20000.0, 10000.0),
            "a completed free-kick execution is already inactive");
}

void checkAuthorityAndRefereeWindow()
{
    const policy::PlanAuthority low{3, 3, 1, 10};
    const policy::PlanAuthority high{4, 4, 1, 11};
    require(policy::authorityBetter(high, low),
            "higher leader term wins plan authority");
    require(!policy::authorityBetter(low, high),
            "lower leader term cannot replace a plan");
    const policy::PlanAuthority restarted{3, 3, 1, 11};
    const policy::PlanAuthority oldBootHighEvent{3, 3, 99, 10};
    require(policy::authorityBetter(restarted, oldBootHighEvent),
            "a restarted proposer boot outranks an old boot event counter");
    require(policy::packetInRestartWindow(101, 100, 104),
            "plan packet from current restart is accepted");
    require(policy::packetInRestartWindow(99, 100, 104),
            "small packet skew before local STOP is accepted");
    require(!policy::packetInRestartWindow(80, 100, 104),
            "stale plan packet is rejected");
    require(policy::packetNewer(3, 2),
            "packet sequence advances normally");
    require(policy::packetNewer(0, 255),
            "packet sequence wraps at 255");
    require(!policy::packetNewer(2, 2),
            "duplicate packet is rejected");
    require(!policy::packetNewer(2, 100),
            "old packet is rejected");
}

} // namespace

int main()
{
    checkGeometry({9.0, 6.0});
    checkGeometry({14.16, 9.22});
    checkGeometry({22.003, 14.126});
    checkOpponentAwareSelection();
    checkLifecycle();
    checkAuthorityAndRefereeWindow();
    std::cout << "freekick policy tests passed\n";
    return 0;
}
