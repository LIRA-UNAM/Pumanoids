#include <cstdint>
#include <iostream>

#include <booster/robot/b1/b1_loco_api.hpp>

namespace {

int fail(const char *message)
{
    std::cerr << message << '\n';
    return 1;
}

} // namespace

int main()
{
    using booster::robot::RobotMode;
    using booster::robot::b1::ChangeModeParameter;
    using booster::robot::b1::GetUpParameter;
    using booster::robot::b1::GetUpVersion;
    using booster::robot::b1::GetTrainedTrajStatusResponse;
    using booster::robot::b1::LocoApiId;
    using booster::robot::b1::MoveParameter;
    using booster::robot::b1::RotateHeadParameter;
    using booster::robot::b1::WaveHandParameter;

    auto trainedTrajStatus = nlohmann::json{{"status", 1}, {"traj_id", "demo"}};
    GetTrainedTrajStatusResponse trainedTrajResponse;
    trainedTrajResponse.FromJson(trainedTrajStatus);
    if (static_cast<std::int64_t>(LocoApiId::kGetTrainedTrajStatus) != 2047 ||
        trainedTrajResponse.status_ != booster::robot::TrainedTrajStatus::kRunning ||
        trainedTrajResponse.traj_id_ != "demo") {
        return fail("Booster Robotics release SDK marker API is unavailable");
    }
    if (static_cast<std::int64_t>(LocoApiId::kGetMode) != 2017 ||
        static_cast<int>(RobotMode::kPrepare) != 1 ||
        static_cast<int>(RobotMode::kWalking) != 2 ||
        static_cast<int>(RobotMode::kSoccer) != 4) {
        return fail("recovery mode protocol changed");
    }

    auto modeBody = nlohmann::json{{"mode", 4}};
    booster::robot::b1::GetModeResponse modeResponse;
    modeResponse.FromJson(modeBody);
    if (modeResponse.mode_ != RobotMode::kSoccer) {
        return fail("GetMode response protocol changed");
    }

    const auto changeMode = ChangeModeParameter(RobotMode::kPrepare).ToJson();
    const auto move = MoveParameter(1.0F, 2.0F, -3.0F).ToJson();
    const auto rotateHead = RotateHeadParameter(0.5F, -0.5F).ToJson();
    const auto waveHand = WaveHandParameter(
        booster::robot::b1::kRightHand,
        booster::robot::b1::kHandOpen).ToJson();
    if (static_cast<std::int64_t>(LocoApiId::kChangeMode) != 2000 ||
        static_cast<std::int64_t>(LocoApiId::kMove) != 2001 ||
        static_cast<std::int64_t>(LocoApiId::kRotateHead) != 2004 ||
        static_cast<std::int64_t>(LocoApiId::kWaveHand) != 2005 ||
        static_cast<std::int64_t>(LocoApiId::kShoot) != 2024 ||
        changeMode.at("mode").get<int>() != 1 ||
        move.at("vx").get<float>() != 1.0F ||
        move.at("vy").get<float>() != 2.0F ||
        move.at("vyaw").get<float>() != -3.0F ||
        rotateHead.at("pitch").get<float>() != 0.5F ||
        rotateHead.at("yaw").get<float>() != -0.5F ||
        waveHand.at("hand_index").get<int>() != 1 ||
        waveHand.at("hand_action").get<int>() != 0) {
        return fail("basic motion request protocol changed");
    }

    const auto getUpV1 = GetUpParameter(GetUpVersion::kV1).ToJson();
    const auto getUpV2 = GetUpParameter(GetUpVersion::kV2).ToJson();
    if (static_cast<std::int64_t>(LocoApiId::kGetUp) != 2008 ||
        getUpV1.at("version").get<int>() != 0 ||
        getUpV2.at("version").get<int>() != 1) {
        return fail("GetUp request protocol changed");
    }

    const booster::robot::b1::VisualKickParameter visualKick(
        true, booster::robot::b1::VisualKickVersion::kV2);
    const auto visualKickBody = visualKick.ToJson();
    if (static_cast<std::int64_t>(LocoApiId::kVisualKick) != 2038 ||
        !visualKickBody.at("start").get<bool>() ||
        visualKickBody.at("version").get<int>() != 1) {
        return fail("VisualKick request protocol changed");
    }

    return 0;
}
