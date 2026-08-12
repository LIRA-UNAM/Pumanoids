#include <chrono>
#include <cstdint>
#include <thread>

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include "booster_vision/base/data_syncer.hpp"

namespace booster_vision {
namespace {

DepthDataBlock MakeDepth(double timestamp, uint16_t value = 1000) {
    return DepthDataBlock(cv::Mat(2, 2, CV_16UC1, cv::Scalar(value)), timestamp);
}

TEST(DataSyncerTest, LeavesDepthEmptyWhenNoValidSampleExists) {
    DataSyncer syncer(true);
    syncer.AddDepth(DepthDataBlock(cv::Mat(), 10.0));
    syncer.AddDepth(MakeDepth(0.0));

    const auto synced = syncer.getSyncedDataBlock(
        ColorDataBlock(cv::Mat(2, 2, CV_8UC3), 10.0));

    EXPECT_TRUE(synced.depth_data.data.empty());
    EXPECT_DOUBLE_EQ(synced.depth_data.timestamp, 0.0);
}

TEST(DataSyncerTest, SelectsNearestDepthRegardlessOfArrivalOrder) {
    DataSyncer syncer(true);
    syncer.AddDepth(MakeDepth(10.01, 1010));
    syncer.AddDepth(MakeDepth(9.95, 950));

    const auto synced = syncer.getSyncedDataBlock(
        ColorDataBlock(cv::Mat(2, 2, CV_8UC3), 10.0));

    EXPECT_DOUBLE_EQ(synced.depth_data.timestamp, 10.01);
    EXPECT_EQ(synced.depth_data.data.at<uint16_t>(0, 0), 1010);
}

TEST(DataSyncerTest, WaitsForMatchingDepthInsteadOfUsingPreviousFrame) {
    DataSyncer syncer(true, 0.0, 0.0, 30.0);
    syncer.AddDepth(MakeDepth(9.95, 950));

    std::thread producer([&syncer]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        syncer.AddDepth(MakeDepth(10.0, 1000));
    });
    const auto synced = syncer.getSyncedDataBlock(
        ColorDataBlock(cv::Mat(2, 2, CV_8UC3), 10.0));
    producer.join();

    EXPECT_DOUBLE_EQ(synced.depth_data.timestamp, 10.0);
    EXPECT_EQ(synced.depth_data.data.at<uint16_t>(0, 0), 1000);
}

TEST(DataSyncerTest, DropsOldDepthEpochAfterTimestampReset) {
    DataSyncer syncer(true);
    for (int index = 0; index < kDepthBufferLength; ++index) {
        syncer.AddDepth(MakeDepth(100.0 + index * 0.01, 2000));
    }

    syncer.AddDepth(MakeDepth(1.0, 1000));
    const auto synced = syncer.getSyncedDataBlock(
        ColorDataBlock(cv::Mat(2, 2, CV_8UC3), 1.0));

    EXPECT_DOUBLE_EQ(synced.depth_data.timestamp, 1.0);
    EXPECT_EQ(synced.depth_data.data.at<uint16_t>(0, 0), 1000);
}

} // namespace
} // namespace booster_vision
