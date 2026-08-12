#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace robot_obstacle_policy {

struct Position2D
{
    double x = 0.0;
    double y = 0.0;
};

struct MatchCandidate
{
    double distance = std::numeric_limits<double>::infinity();
    std::size_t detectionIndex = 0;
    std::size_t trackIndex = 0;
};

inline std::vector<std::vector<std::size_t>> clusterNearbyPositions(
    const std::vector<Position2D> &positions,
    double mergeDistance)
{
    std::vector<std::vector<std::size_t>> clusters;
    if (!std::isfinite(mergeDistance) || mergeDistance <= 0.0) {
        clusters.reserve(positions.size());
        for (std::size_t index = 0; index < positions.size(); ++index) {
            clusters.push_back({index});
        }
        return clusters;
    }

    std::vector<std::size_t> parent(positions.size());
    for (std::size_t index = 0; index < parent.size(); ++index) {
        parent[index] = index;
    }
    const auto findRoot = [&](std::size_t index, const auto &self) -> std::size_t {
        if (parent[index] == index) return index;
        parent[index] = self(parent[index], self);
        return parent[index];
    };

    for (std::size_t left = 0; left < positions.size(); ++left) {
        if (!std::isfinite(positions[left].x) ||
            !std::isfinite(positions[left].y)) {
            continue;
        }
        for (std::size_t right = left + 1; right < positions.size(); ++right) {
            if (!std::isfinite(positions[right].x) ||
                !std::isfinite(positions[right].y) ||
                std::hypot(
                    positions[left].x - positions[right].x,
                    positions[left].y - positions[right].y) >= mergeDistance) {
                continue;
            }
            const std::size_t leftRoot = findRoot(left, findRoot);
            const std::size_t rightRoot = findRoot(right, findRoot);
            if (leftRoot != rightRoot) parent[rightRoot] = leftRoot;
        }
    }

    std::vector<std::size_t> roots;
    for (std::size_t index = 0; index < positions.size(); ++index) {
        const std::size_t root = findRoot(index, findRoot);
        const auto existing = std::find(roots.begin(), roots.end(), root);
        if (existing == roots.end()) {
            roots.push_back(root);
            clusters.push_back({index});
        } else {
            clusters[static_cast<std::size_t>(existing - roots.begin())]
                .push_back(index);
        }
    }
    return clusters;
}

inline uint32_t nextDepthSeenCount(uint32_t previousCount)
{
    return previousCount == std::numeric_limits<uint32_t>::max()
        ? previousCount : previousCount + 1;
}

inline uint32_t nextDepthMissedCount(uint32_t previousCount)
{
    return nextDepthSeenCount(previousCount);
}

// A cell is retained for clearFrames - 1 misses. A value of one therefore
// means immediate removal, while the default value two tolerates one hole.
inline bool retainMissedDepthCell(
    uint32_t previousMissedCount, int clearFrames)
{
    if (clearFrames <= 1) return false;
    return nextDepthMissedCount(previousMissedCount) <
        static_cast<uint32_t>(clearFrames);
}

inline std::vector<int> matchNearestTracks(
    const std::vector<Position2D> &detections,
    const std::vector<Position2D> &tracks,
    double maxDistance)
{
    std::vector<int> matches(detections.size(), -1);
    if (!std::isfinite(maxDistance) || maxDistance <= 0.0) return matches;

    // T2 normally has at most five simultaneous robot detections. Exhaustive
    // assignment for this small set minimizes the total displacement instead
    // of making a locally nearest greedy choice at a crossing. Keep a bounded
    // greedy fallback for malformed/over-sized detector output.
    if (detections.size() <= 8 && tracks.size() <= 8) {
        std::vector<int> bestMatches(detections.size(), -1);
        std::vector<int> currentMatches(detections.size(), -1);
        std::vector<bool> usedTracks(tracks.size(), false);
        std::size_t bestMatchedCount = 0;
        double bestCost = std::numeric_limits<double>::infinity();

        const auto isBetterTie = [&](const std::vector<int> &lhs,
                                     const std::vector<int> &rhs) {
            for (std::size_t i = 0; i < lhs.size(); ++i) {
                const int left = lhs[i] < 0
                    ? static_cast<int>(tracks.size()) : lhs[i];
                const int right = rhs[i] < 0
                    ? static_cast<int>(tracks.size()) : rhs[i];
                if (left != right) return left < right;
            }
            return false;
        };

        std::function<void(std::size_t, std::size_t, double)> search;
        search = [&](std::size_t detectionIndex,
                     std::size_t matchedCount,
                     double cost) {
            const std::size_t remainingDetections =
                detections.size() - detectionIndex;
            const std::size_t remainingTracks =
                tracks.size() - matchedCount;
            if (matchedCount +
                    std::min(remainingDetections, remainingTracks) <
                bestMatchedCount) {
                return;
            }
            if (detectionIndex == detections.size()) {
                if (matchedCount > bestMatchedCount ||
                    (matchedCount == bestMatchedCount &&
                     (cost < bestCost - 1e-12 ||
                      (std::fabs(cost - bestCost) <= 1e-12 &&
                       isBetterTie(currentMatches, bestMatches))))) {
                    bestMatchedCount = matchedCount;
                    bestCost = cost;
                    bestMatches = currentMatches;
                }
                return;
            }

            const auto &detection = detections[detectionIndex];
            if (std::isfinite(detection.x) && std::isfinite(detection.y)) {
                // Explore valid matches first. A full assignment is normally
                // found immediately, allowing the cardinality bound above to
                // prune branches that leave matchable detections unused.
                for (std::size_t trackIndex = 0;
                     trackIndex < tracks.size(); ++trackIndex) {
                    if (usedTracks[trackIndex]) continue;
                    const auto &track = tracks[trackIndex];
                    if (!std::isfinite(track.x) || !std::isfinite(track.y)) {
                        continue;
                    }
                    const double distance = std::hypot(
                        detection.x - track.x, detection.y - track.y);
                    if (!std::isfinite(distance) || distance >= maxDistance) {
                        continue;
                    }
                    usedTracks[trackIndex] = true;
                    currentMatches[detectionIndex] =
                        static_cast<int>(trackIndex);
                    search(detectionIndex + 1, matchedCount + 1,
                           cost + distance);
                    usedTracks[trackIndex] = false;
                }
            }

            // Keep an unmatched branch for invalid detections, missing gates,
            // or frames containing more detections than remembered tracks.
            currentMatches[detectionIndex] = -1;
            search(detectionIndex + 1, matchedCount, cost);
        };
        search(0, 0, 0.0);
        return bestMatches;
    }

    std::vector<MatchCandidate> candidates;
    candidates.reserve(detections.size() * tracks.size());
    for (std::size_t detectionIndex = 0;
         detectionIndex < detections.size(); ++detectionIndex) {
        const auto &detection = detections[detectionIndex];
        if (!std::isfinite(detection.x) || !std::isfinite(detection.y)) continue;
        for (std::size_t trackIndex = 0; trackIndex < tracks.size(); ++trackIndex) {
            const auto &track = tracks[trackIndex];
            if (!std::isfinite(track.x) || !std::isfinite(track.y)) continue;
            const double distance = std::hypot(
                detection.x - track.x, detection.y - track.y);
            if (distance < maxDistance) {
                candidates.push_back({distance, detectionIndex, trackIndex});
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const auto &lhs, const auto &rhs) {
        if (lhs.distance != rhs.distance) return lhs.distance < rhs.distance;
        if (lhs.detectionIndex != rhs.detectionIndex) {
            return lhs.detectionIndex < rhs.detectionIndex;
        }
        return lhs.trackIndex < rhs.trackIndex;
    });

    std::vector<bool> trackMatched(tracks.size(), false);
    for (const auto &candidate : candidates) {
        if (matches[candidate.detectionIndex] >= 0 ||
            trackMatched[candidate.trackIndex]) {
            continue;
        }
        matches[candidate.detectionIndex] =
            static_cast<int>(candidate.trackIndex);
        trackMatched[candidate.trackIndex] = true;
    }
    return matches;
}

inline bool coversDepthPoint(
    Position2D robotCenter,
    Position2D depthPoint,
    double centerYaw,
    double robotRadius,
    double fusionMargin,
    bool footprintValid,
    double footprintWidth)
{
    if (!std::isfinite(robotCenter.x) || !std::isfinite(robotCenter.y) ||
        !std::isfinite(depthPoint.x) || !std::isfinite(depthPoint.y) ||
        !std::isfinite(centerYaw) || !std::isfinite(robotRadius) ||
        !std::isfinite(fusionMargin) || robotRadius < 0.0 ||
        fusionMargin < 0.0) {
        return false;
    }

    const double dx = depthPoint.x - robotCenter.x;
    const double dy = depthPoint.y - robotCenter.y;
    if (!footprintValid || !std::isfinite(footprintWidth) ||
        footprintWidth <= 0.0) {
        return std::hypot(dx, dy) < fusionMargin;
    }

    const double halfWidth = std::max(0.05, 0.5 * footprintWidth);
    const double longitudinal = dx * std::cos(centerYaw) +
        dy * std::sin(centerYaw);
    const double lateral = std::fabs(
        -dx * std::sin(centerYaw) + dy * std::cos(centerYaw));
    return longitudinal > -fusionMargin &&
        longitudinal < robotRadius + fusionMargin &&
        lateral < halfWidth + fusionMargin;
}

} // namespace robot_obstacle_policy
