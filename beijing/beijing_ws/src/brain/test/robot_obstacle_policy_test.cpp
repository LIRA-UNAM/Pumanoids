#include "robot_obstacle_policy.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace policy = robot_obstacle_policy;

void expect(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    const std::vector<policy::Position2D> tracks{{1.0, -0.5}, {1.0, 0.5}};
    const std::vector<policy::Position2D> detections{{1.02, 0.48}, {0.98, -0.48}};
    const auto matches = policy::matchNearestTracks(detections, tracks, 0.7);
    expect(matches.size() == 2 && matches[0] == 1 && matches[1] == 0,
           "detection output order changed robot-track assignment");
    expect(matches[0] != matches[1],
           "one track was assigned to more than one detection");

    const auto globalMatches = policy::matchNearestTracks(
        {{0.4, 0.0}, {-0.5, 0.0}},
        {{0.0, 0.0}, {1.0, 0.0}},
        2.0);
    expect(globalMatches.size() == 2 &&
               globalMatches[0] == 1 && globalMatches[1] == 0,
           "greedy local match replaced the lower-cost global assignment");

    std::vector<policy::Position2D> denseTracks;
    std::vector<policy::Position2D> denseDetections;
    for (int i = 0; i < 8; ++i) {
        denseTracks.push_back({static_cast<double>(i), 0.0});
        denseDetections.push_back({static_cast<double>(7 - i) + 0.01, 0.0});
    }
    const auto denseMatches = policy::matchNearestTracks(
        denseDetections, denseTracks, 10.0);
    std::vector<bool> denseTrackUsed(denseTracks.size(), false);
    for (std::size_t i = 0; i < denseMatches.size(); ++i) {
        expect(denseMatches[i] >= 0,
               "dense assignment unexpectedly dropped a detection");
        expect(!denseTrackUsed[static_cast<std::size_t>(denseMatches[i])],
               "dense assignment reused a track");
        denseTrackUsed[static_cast<std::size_t>(denseMatches[i])] = true;
        expect(denseMatches[i] == 7 - static_cast<int>(i),
               "dense assignment did not minimize global displacement");
    }

    const auto unmatched = policy::matchNearestTracks(
        {{3.0, 0.0}}, tracks, 0.7);
    expect(unmatched.size() == 1 && unmatched[0] == -1,
           "out-of-range detection was matched to a robot track");

    const auto clusters = policy::clusterNearbyPositions(
        {{1.0, 0.0}, {1.08, 0.04}, {2.0, 0.0}, {2.20, 0.0}}, 0.25);
    expect(clusters.size() == 2 && clusters[0].size() == 2 &&
               clusters[1].size() == 2,
           "nearby duplicate robot positions were not clustered");
    const auto separate = policy::clusterNearbyPositions(
        {{1.0, 0.0}, {1.31, 0.0}}, 0.30);
    expect(separate.size() == 2,
           "spatial merge removed distinct robot positions");

    const policy::Position2D center{2.0, 0.0};
    expect(policy::coversDepthPoint(
               center, {2.2, 0.2}, 0.0, 0.6, 0.2, true, 0.6),
           "depth point on the recognized robot was not fused");
    expect(!policy::coversDepthPoint(
               center, {2.0, 0.8}, 0.0, 0.6, 0.2, true, 0.6),
           "adjacent depth obstacle outside the footprint was incorrectly fused");
    expect(policy::coversDepthPoint(
               center, {2.1, 0.0}, 0.0, 0.6, 0.2, false, 0.0),
           "distance fallback did not fuse a depth point without a footprint");

    expect(policy::nextDepthSeenCount(1) == 2,
           "depth confirmation count did not advance");
    expect(policy::retainMissedDepthCell(0, 2),
           "first depth hole was not retained");
    expect(!policy::retainMissedDepthCell(1, 2),
           "depth cell survived beyond its clear-frame budget");

    std::cout << "robot obstacle policy tests passed\n";
    return 0;
}
