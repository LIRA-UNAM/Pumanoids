#include "booster_vision/base/data_syncer.hpp"

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iterator>
#include <regex>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>

namespace {

booster_vision::Pose InterpolatePose(const booster_vision::Pose &before,
                                     const booster_vision::Pose &after,
                                     double alpha) {
    const auto before_t = before.getTranslationVec();
    const auto after_t = after.getTranslationVec();
    const auto before_q = before.getQuaternionVec();
    const auto after_q = after.getQuaternionVec();

    tf2::Quaternion q_before(before_q[0], before_q[1], before_q[2], before_q[3]);
    tf2::Quaternion q_after(after_q[0], after_q[1], after_q[2], after_q[3]);
    q_before.normalize();
    q_after.normalize();
    tf2::Quaternion q_interpolated =
        q_before.slerp(q_after, static_cast<tf2Scalar>(alpha));
    q_interpolated.normalize();

    const auto lerp = [alpha](float a, float b) {
        return static_cast<float>(
            static_cast<double>(a) + alpha * static_cast<double>(b - a));
    };
    return booster_vision::Pose(
        lerp(before_t[0], after_t[0]),
        lerp(before_t[1], after_t[1]),
        lerp(before_t[2], after_t[2]),
        static_cast<float>(q_interpolated.x()),
        static_cast<float>(q_interpolated.y()),
        static_cast<float>(q_interpolated.z()),
        static_cast<float>(q_interpolated.w()));
}

} // namespace

namespace booster_vision {

void DataSyncer::LoadData(const std::string &data_dir) {
    if (!std::filesystem::is_directory(data_dir)) {
        throw std::runtime_error("data directory does not exist: " + data_dir);
    }
    data_dir_ = data_dir;
    // list all files in the directory
    std::vector<std::string> files;
    for (const auto &entry : std::filesystem::directory_iterator(data_dir)) {
        files.push_back(entry.path().filename());
    }
    // regex for matching file names color_xx.xx.jpg, parse xx.xx as timestamp
    std::regex color_file_regex("color_([0-9]+\\.[0-9]+)\\.jpg");
    std::smatch m;
    for (const std::string &file : files) {
        if (std::regex_match(file, m, color_file_regex)) {
            double timestamp = std::stod(m[1].str());
            // std::cout << std::fixed << timestamp << std::endl;
            std::cout << "found timestamp : " << std::fixed << timestamp << std::endl;
            time_stamp_list_.push_back(timestamp);
        }
    }
    std::sort(time_stamp_list_.begin(), time_stamp_list_.end());
    data_index_ = 0;
    std::cout << "loaded " << time_stamp_list_.size() << " data" << std::endl;
}

void DataSyncer::AddDepth(const DepthDataBlock &depth_data) {
    if (!enable_depth_ || depth_data.data.empty() ||
        !std::isfinite(depth_data.timestamp) || depth_data.timestamp <= 0.0) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(depth_buffer_mutex_);
        constexpr double kTimestampResetThresholdSec = 1.0;
        if (!depth_buffer_.empty() &&
            depth_data.timestamp <
                depth_buffer_.back().timestamp - kTimestampResetThresholdSec) {
            depth_buffer_.clear();
        }
        const auto insertion_point = std::lower_bound(
            depth_buffer_.begin(), depth_buffer_.end(), depth_data.timestamp,
            [](const DepthDataBlock &buffered, double timestamp) {
                return buffered.timestamp < timestamp;
            });
        constexpr double kDuplicateTimestampToleranceSec = 1e-9;
        if (insertion_point != depth_buffer_.end() &&
            std::abs(insertion_point->timestamp - depth_data.timestamp) <=
                kDuplicateTimestampToleranceSec) {
            *insertion_point = depth_data;
        } else {
            depth_buffer_.insert(insertion_point, depth_data);
            while (depth_buffer_.size() > kDepthBufferLength) {
                depth_buffer_.pop_front();
            }
        }
    }
    depth_buffer_cv_.notify_all();
}

void DataSyncer::AddPose(const PoseDataBlock &pose_data) {
    if (!std::isfinite(pose_data.timestamp) || pose_data.timestamp <= 0.0) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(pose_buffer_mutex_);
        const auto insert_pose = [&](const PoseDataBlock &sample) {
            const auto insertion_point = std::lower_bound(
                pose_buffer_.begin(), pose_buffer_.end(), sample.timestamp,
                [](const PoseDataBlock &buffered, double timestamp) {
                    return buffered.timestamp < timestamp;
                });
            constexpr double kDuplicateTimestampToleranceSec = 1e-9;
            if (insertion_point != pose_buffer_.end() &&
                std::abs(insertion_point->timestamp - sample.timestamp) <=
                    kDuplicateTimestampToleranceSec) {
                *insertion_point = sample;
            } else {
                pose_buffer_.insert(insertion_point, sample);
                while (pose_buffer_.size() > kPoseBufferLength) {
                    pose_buffer_.pop_front();
                }
            }
        };

        constexpr double kTimestampResetThresholdSec = 1.0;
        constexpr size_t kTimestampResetConfirmationSamples = 3;
        const bool large_backward_jump = !pose_buffer_.empty() &&
            pose_data.timestamp <
                pose_buffer_.back().timestamp - kTimestampResetThresholdSec;
        if (large_backward_jump) {
            const bool continues_candidate_epoch =
                pose_time_reset_candidates_.empty() ||
                (pose_data.timestamp >
                     pose_time_reset_candidates_.back().timestamp + 1e-9 &&
                 pose_data.timestamp -
                         pose_time_reset_candidates_.back().timestamp <=
                     kTimestampResetThresholdSec);
            if (!continues_candidate_epoch) {
                pose_time_reset_candidates_.clear();
            }
            pose_time_reset_candidates_.push_back(pose_data);
            if (pose_time_reset_candidates_.size() <
                kTimestampResetConfirmationSamples) {
                return;
            }

            pose_buffer_.clear();
            for (const auto &candidate : pose_time_reset_candidates_) {
                insert_pose(candidate);
            }
            pose_time_reset_candidates_.clear();
        } else {
            pose_time_reset_candidates_.clear();
            insert_pose(pose_data);
        }
    }
    pose_buffer_cv_.notify_all();
}

SyncedDataBlock DataSyncer::getSyncedDataBlock() {
    std::string color_file_path = data_dir_ + "/color_" + std::to_string(time_stamp_list_[data_index_]) + ".jpg";
    std::string depth_file_path = data_dir_ + "/depth_" + std::to_string(time_stamp_list_[data_index_]) + ".png";
    std::string pose_file_path = data_dir_ + "/pose_" + std::to_string(time_stamp_list_[data_index_]) + ".yaml";
    double timestamp = time_stamp_list_[data_index_];
    SyncedDataBlock synced_data;

    if (std::filesystem::exists(color_file_path)) {
        ColorDataBlock color_data;
        color_data.data = cv::imread(color_file_path);
        color_data.timestamp = timestamp;
        synced_data.color_data = color_data;
    } else {
        std::cout << "color file not found: " << color_file_path << std::endl;
    }

    if (enable_depth_) {
        if (std::filesystem::exists(depth_file_path)) {
            DepthDataBlock depth_data;
            depth_data.data = cv::imread(depth_file_path, cv::IMREAD_ANYDEPTH);
            depth_data.timestamp = timestamp;
            synced_data.depth_data = depth_data;
        } else {
            std::cout << "depth file not found: " << depth_file_path << std::endl;
        }
    }

    if (std::filesystem::exists(pose_file_path)) {
        PoseDataBlock pose_data;
        YAML::Node pose_node = YAML::LoadFile(pose_file_path);
        if (pose_node["pose"]) {
            pose_data.data = pose_node["pose"].as<Pose>();
        } else {
            pose_data.data = pose_node.as<Pose>();
        }
        pose_data.timestamp = timestamp;
        synced_data.pose_data = pose_data;
        synced_data.pose_valid = true;
    } else {
        std::cout << "pose file not found: " << pose_file_path << std::endl;
    }

    data_index_ = (data_index_ + 1) % time_stamp_list_.size();
    if (data_index_ == 0) {
        std::cout << "looped all data, reset index to 0" << std::endl;
    }
    return synced_data;
}

SyncedDataBlock DataSyncer::getSyncedDataBlock(const ColorDataBlock &color_data) {
    SyncedDataBlock synced_data;
    synced_data.color_data = color_data;

    double color_timestamp = color_data.timestamp;
    if (enable_depth_) {
        std::unique_lock<std::mutex> lock(depth_buffer_mutex_);
        const bool matching_depth_may_arrive = depth_buffer_.empty() ||
            depth_buffer_.back().timestamp < color_timestamp;
        if (depth_wait_timeout_ms_ > 0.0 && matching_depth_may_arrive) {
            depth_buffer_cv_.wait_for(
                lock,
                std::chrono::duration<double, std::milli>(
                    depth_wait_timeout_ms_),
                [&]() {
                    return !depth_buffer_.empty() &&
                        depth_buffer_.back().timestamp >= color_timestamp;
                });
        }

        if (!depth_buffer_.empty()) {
            const auto nearest = std::min_element(
                depth_buffer_.begin(), depth_buffer_.end(),
                [color_timestamp](const DepthDataBlock &lhs,
                                  const DepthDataBlock &rhs) {
                    return std::abs(lhs.timestamp - color_timestamp) <
                        std::abs(rhs.timestamp - color_timestamp);
                });
            synced_data.depth_data = *nearest;
        }
    }

    {
        std::unique_lock<std::mutex> lock(pose_buffer_mutex_);
        const bool target_can_be_bracketed_soon = pose_buffer_.empty() ||
            (color_timestamp > pose_buffer_.back().timestamp &&
             color_timestamp - pose_buffer_.back().timestamp <=
                 pose_interpolation_max_gap_sec_);
        if (pose_wait_timeout_ms_ > 0.0 && target_can_be_bracketed_soon) {
            pose_buffer_cv_.wait_for(
                lock, std::chrono::duration<double, std::milli>(pose_wait_timeout_ms_),
                [&]() {
                    return !pose_buffer_.empty() &&
                        pose_buffer_.back().timestamp >= color_timestamp;
                });
        }

        if (!pose_buffer_.empty()) {
            const auto after = std::lower_bound(
                pose_buffer_.begin(), pose_buffer_.end(), color_timestamp,
                [](const PoseDataBlock &sample, double timestamp) {
                    return sample.timestamp < timestamp;
                });

            if (after == pose_buffer_.begin()) {
                synced_data.pose_data = *after;
                synced_data.pose_data.nearest_source_diff =
                    std::abs(after->timestamp - color_timestamp);
            } else if (after == pose_buffer_.end()) {
                synced_data.pose_data = pose_buffer_.back();
                synced_data.pose_data.nearest_source_diff =
                    std::abs(pose_buffer_.back().timestamp - color_timestamp);
            } else {
                const auto before = std::prev(after);
                const double before_diff = color_timestamp - before->timestamp;
                const double after_diff = after->timestamp - color_timestamp;
                const double interpolation_span = after->timestamp - before->timestamp;
                constexpr double kExactTimestampToleranceSec = 1e-9;

                if (std::abs(after->timestamp - color_timestamp) <=
                    kExactTimestampToleranceSec) {
                    synced_data.pose_data = *after;
                    synced_data.pose_data.nearest_source_diff = 0.0;
                } else if (pose_interpolation_max_gap_sec_ > 0.0 &&
                           interpolation_span > kExactTimestampToleranceSec &&
                           interpolation_span <= pose_interpolation_max_gap_sec_) {
                    const double alpha = before_diff / interpolation_span;
                    synced_data.pose_data = PoseDataBlock(
                        InterpolatePose(before->data, after->data, alpha),
                        color_timestamp);
                    synced_data.pose_data.nearest_source_diff =
                        std::min(before_diff, after_diff);
                    synced_data.pose_data.interpolation_span = interpolation_span;
                    synced_data.pose_data.interpolated = true;
                } else {
                    const auto nearest = before_diff <= after_diff ? before : after;
                    synced_data.pose_data = *nearest;
                    synced_data.pose_data.nearest_source_diff =
                        std::min(before_diff, after_diff);
                }
            }
            synced_data.pose_valid = true;
        }
    }
    return synced_data;
}

} // namespace booster_vision
