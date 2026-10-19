// Runs an mjlab-trained velocity-tracking (walking) policy on the Booster T1.
//
// Inputs:  booster_interface/LowState (trunk IMU + serial motor states), a
//          velocity command from one source: the Booster remote controller
//          (use_remote, booster_interface/RemoteControllerState),
//          geometry_msgs/Twist on cmd_vel_topic (use_cmd_vel) or the ROS
//          parameters cmd_vx/cmd_vy/cmd_wz set by the GUI (both off).
// Output:  booster_interface/LowCmd (serial, one MotorCmd per joint) on
//          low_cmd_topic, only while enable_lowcmd is set (the robot must be in
//          Custom mode: ~/enter_custom_mode or the Booster app).
//
// The policy (task Pumas-Velocity-*-Booster-T1-PPO) controls every joint but
// the neck with an absolute position action: target = default_joint_pos +
// action_scale * a. The neck is held at held_joint_targets (0, as in training,
// where its own PD holds HOME_KEYFRAME). Each commanded joint gets kp/kd (the
// training gains) and its target clamped to the joint limits; every other joint,
// and every joint while IDLE or without fresh LowState, is damped (kp = 0,
// kd = damping_kd, q = measured).
//
// Pinned observations: the joint_pos / joint_vel observation of a joint the
// policy cannot rely on is reported as if it sat at its hold target, at rest,
// as in training. That covers every non-commanded joint (limp under damping),
// every motor reporting error flags (MotorState.reserve[0] != 0; latched for the
// whole run and damped) and the manual obs_pinned_joints list. Start is refused
// while a leg motor reports errors.
//
// States:
//   IDLE     every joint damped
//   HOMING   slow cosine move to home_pose, then HOLDING
//   HOLDING  the last target is held
//   RAMPING  policy running; the command blends from the start pose into the
//            policy targets over start_ramp_s
//   RUNNING  policy targets
// Models: the *.onnx files of models_dir (default: the folder of policy_path)
// are listed in ~/status for the GUI's dropdown, which loads one by setting
// policy_path (only while the policy is stopped; a model whose input/output
// sizes do not match is rejected and the current one stays loaded; metadata
// that disagrees with the parameters is reported as a warning).
//
// The policy can only be started while the IMU reports the robot upright
// (tilt <= max_start_tilt_deg). The tilt is computed continuously from the IMU
// and published in ~/status so the GUI can disable Start.
//
// Remote controller: with use_remote the left stick commands vx (up/down) and vy
// (left/right) and the right stick wz (left/right), scaled so that full
// deflection reaches the velocity limits (remote_deadzone gives an exact 0).
// With remote_start, pressing START (+) runs the same Start as the ~/start
// service, with all its checks (upright, leg motors healthy, fresh data, model
// loaded); with remote_home, pressing BACK (-) runs the same Home as ~/home.
// A refused press is logged and reported in ~/status.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_cmd.hpp"
#include "booster_interface/msg/remote_controller_state.hpp"
#include "booster_interface/srv/rpc_service.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "t1_ppowalk/motion.hpp"
#include "t1_ppowalk/observation.hpp"
#include "t1_ppowalk/onnx_policy.hpp"

namespace t1_ppowalk
{

using SteadyClock = std::chrono::steady_clock;
using booster_interface::msg::LowCmd;
using booster_interface::msg::LowState;
using booster_interface::msg::RemoteControllerState;
using booster_interface::srv::RpcService;

// Booster loco API: ChangeMode (body {"mode": <RobotMode>}).
constexpr int64_t kChangeModeApi = 2000;

// RemoteControllerState buttons, in the order reported in ~/status.
constexpr int kNumRemoteButtons = 12;
const char * const kRemoteButtonNames[kNumRemoteButtons] = {
  "A", "B", "X", "Y", "LB", "RB", "LT", "RT", "LS", "RS", "BACK", "START"};

class T1PpowalkPolicyNode : public rclcpp::Node
{
public:
  T1PpowalkPolicyNode()
  : Node("t1_ppowalk_policy_node")
  {
    policy_path_ = declare_parameter<std::string>("policy_path", "");
    // Folder of selectable models (GUI dropdown); empty: the folder of
    // policy_path (~/t1_ppowalk_models without one).
    models_dir_param_ = declare_parameter<std::string>("models_dir", "");
    joint_names_ =
      declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    action_joint_names_ = declare_parameter<std::vector<std::string>>(
      "action_joint_names", std::vector<std::string>{});
    default_joint_pos_ =
      declare_parameter<std::vector<double>>("default_joint_pos", std::vector<double>{});
    observation_terms_ = declare_parameter<std::vector<std::string>>(
      "observation_terms", std::vector<std::string>{});
    observation_scales_ =
      declare_parameter<std::vector<double>>("observation_scales", std::vector<double>{});
    action_scale_ = declare_parameter<std::vector<double>>("action_scale", std::vector<double>{});
    action_clip_ = declare_parameter<double>("action_clip", 0.0);
    const auto num_obs = declare_parameter<int>("num_obs", 0);
    policy_rate_hz_ = declare_parameter<double>("policy_rate_hz", 50.0);
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 200.0);
    num_threads_ = static_cast<int>(declare_parameter<int>("num_threads", 1));
    data_timeout_ = declare_parameter<double>("data_timeout_s", 0.1);
    // Motor output (replaces the G1 low-level bridge).
    kp_ = declare_parameter<std::vector<double>>("kp", std::vector<double>{});
    kd_ = declare_parameter<std::vector<double>>("kd", std::vector<double>{});
    effort_limit_ = declare_parameter<std::vector<double>>("effort_limit", std::vector<double>{});
    position_lower_ =
      declare_parameter<std::vector<double>>("position_lower", std::vector<double>{});
    position_upper_ =
      declare_parameter<std::vector<double>>("position_upper", std::vector<double>{});
    damping_kd_ = declare_parameter<double>("damping_kd", 1.0);
    max_target_delta_ = declare_parameter<double>("max_target_delta", 0.0);
    enable_lowcmd_ = declare_parameter<bool>("enable_lowcmd", false);
    cmd_type_ = static_cast<int>(declare_parameter<int>("cmd_type", LowCmd::CMD_TYPE_SERIAL));
    custom_mode_ = declare_parameter<int>("custom_mode", 3);
    prepare_mode_ = declare_parameter<int>("prepare_mode", 1);
    // Velocity command (live, from the GUI) and its limits (training ranges).
    cmd_ = {declare_parameter<double>("cmd_vx", 0.0), declare_parameter<double>("cmd_vy", 0.0),
      declare_parameter<double>("cmd_wz", 0.0)};
    cmd_min_ = {declare_parameter<double>("lin_vel_x_min", -1.0),
      declare_parameter<double>("lin_vel_y_min", -1.0),
      declare_parameter<double>("ang_vel_z_min", -0.5)};
    cmd_max_ = {declare_parameter<double>("lin_vel_x_max", 1.0),
      declare_parameter<double>("lin_vel_y_max", 1.0),
      declare_parameter<double>("ang_vel_z_max", 0.5)};
    // Calibration: added to the command (GUI, remote or cmd_vel) to cancel the
    // drift of the real robot (e.g. walking slightly left with vy = 0). Applied
    // only while motion is commanded unless offsets_when_standing.
    cmd_offset_ = {declare_parameter<double>("cmd_vx_offset", 0.0),
      declare_parameter<double>("cmd_vy_offset", 0.0),
      declare_parameter<double>("cmd_wz_offset", 0.0)};
    offsets_when_standing_ = declare_parameter<bool>("offsets_when_standing", false);
    use_cmd_vel_ = declare_parameter<bool>("use_cmd_vel", false);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout_s", 0.5);
    // Remote controller: sticks -> velocity command (use_remote), START (+) ->
    // Start (remote_start), BACK (-) -> Home (remote_home).
    use_remote_ = declare_parameter<bool>("use_remote", true);
    remote_start_ = declare_parameter<bool>("remote_start", true);
    remote_home_ = declare_parameter<bool>("remote_home", true);
    remote_deadzone_ = declare_parameter<double>("remote_deadzone", 0.1);
    remote_timeout_ = declare_parameter<double>("remote_timeout_s", 0.5);
    // Safety / motion.
    max_start_tilt_deg_ = declare_parameter<double>("max_start_tilt_deg", 30.0);
    start_ramp_s_ = declare_parameter<double>("start_ramp_s", 1.0);
    home_pose_ = declare_parameter<std::vector<double>>("home_pose", std::vector<double>{});
    home_max_vel_ = declare_parameter<double>("home_max_vel", 0.3);
    command_joint_names_ = declare_parameter<std::vector<std::string>>(
      "command_joints", std::vector<std::string>{});
    always_commanded_ = declare_parameter<std::vector<std::string>>(
      "always_commanded_joints", std::vector<std::string>{"Waist"});
    held_targets_ =
      declare_parameter<std::vector<double>>("held_joint_targets", std::vector<double>{});
    pinned_names_ = declare_parameter<std::vector<std::string>>(
      "obs_pinned_joints", std::vector<std::string>{});
    detect_offline_ = declare_parameter<bool>("detect_offline", true);
    home_min_duration_ = declare_parameter<double>("home_min_duration_s", 2.0);
    const auto low_state_topic = declare_parameter<std::string>("low_state_topic", "/low_state");
    const auto low_cmd_topic = declare_parameter<std::string>("low_cmd_topic", "joint_ctrl");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto remote_topic =
      declare_parameter<std::string>("remote_topic", "/remote_controller_state");
    const auto rpc_service = declare_parameter<std::string>("rpc_service", "/booster_rpc_service");
    const bool publish_debug = declare_parameter<bool>("publish_debug", true);

    const size_t n = joint_names_.size();
    if (n == 0 || default_joint_pos_.size() != n) {
      throw std::invalid_argument("joint_names and default_joint_pos must be non-empty and match");
    }
    if (home_pose_.empty()) {
      home_pose_ = default_joint_pos_;
    }
    std::string error = check_home(home_pose_);
    if (error.empty()) {
      error = check_motor_params(kp_, kd_, position_lower_, position_upper_);
    }
    if (error.empty() && !effort_limit_.empty() && effort_limit_.size() != n) {
      error = "effort_limit must be empty or have len(joint_names) entries";
    }
    if (error.empty() && cmd_type_ != LowCmd::CMD_TYPE_SERIAL &&
      cmd_type_ != LowCmd::CMD_TYPE_PARALLEL)
    {
      error = "cmd_type must be 0 (parallel) or 1 (serial)";
    }
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    std::unordered_map<std::string, size_t> index;
    for (size_t i = 0; i < n; ++i) {
      index[joint_names_[i]] = i;
    }
    for (const auto & name : action_joint_names_) {
      auto it = index.find(name);
      if (it == index.end()) {
        throw std::invalid_argument("action joint '" + name + "' is not in joint_names");
      }
      action_index_.push_back(it->second);
    }
    if (action_index_.empty()) {
      throw std::invalid_argument("action_joint_names must not be empty");
    }
    action_scale_ = expand_action_scale(action_scale_, error);
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    is_action_.assign(n, false);
    for (size_t idx : action_index_) {
      is_action_[idx] = true;
    }
    is_leg_.assign(n, false);
    for (size_t i = 0; i < n; ++i) {
      const auto & name = joint_names_[i];
      is_leg_[i] = name.find("_Hip_") != std::string::npos ||
        name.find("_Knee_") != std::string::npos || name.find("_Ankle_") != std::string::npos;
    }
    error = check_names(always_commanded_, "always_commanded_joints");
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    if (command_joint_names_.empty()) {
      command_joint_names_ = joint_names_;
    }
    if (held_targets_.empty()) {
      held_targets_.assign(n, 0.0);
    }
    error = check_command_joints(command_joint_names_);
    if (error.empty() && held_targets_.size() != n) {
      error = "held_joint_targets must have len(joint_names) entries";
    }
    if (error.empty()) {
      error = check_names(pinned_names_, "obs_pinned_joints");
    }
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    error = check_remote(use_remote_, use_cmd_vel_, remote_deadzone_, remote_timeout_);
    if (!error.empty()) {
      throw std::invalid_argument(error);
    }
    apply_command_joints(command_joint_names_);
    manual_pin_ = name_mask(pinned_names_);
    offline_now_.assign(n, false);
    session_offline_.assign(n, false);
    obs_builder_ = make_observation_builder(default_joint_pos_, observation_scales_, error);
    if (!obs_builder_) {
      throw std::invalid_argument(error);
    }
    if (num_obs > 0 && static_cast<size_t>(num_obs) != obs_builder_->size()) {
      throw std::invalid_argument(
              "num_obs=" + std::to_string(num_obs) + " but observation_terms give " +
              std::to_string(obs_builder_->size()));
    }
    models_dir_ = resolve_models_dir(models_dir_param_, policy_path_);
    if (!policy_path_.empty()) {
      policy_ = load_policy(policy_path_, error);
      if (!policy_) {
        // Keep running (Home must work); Start is refused until a valid model
        // is loaded through the policy_path parameter.
        RCLCPP_ERROR(get_logger(), "%s", error.c_str());
        policy_path_.clear();
      } else {
        model_warnings_ = check_model_metadata(*policy_);
      }
    }
    if (!policy_) {
      RCLCPP_WARN(get_logger(), "No policy loaded: Home works, Start is refused until a model is loaded");
    }

    state_.joint_pos.assign(n, 0.0);
    state_.joint_vel.assign(n, 0.0);
    action_.assign(action_index_.size(), 0.0f);
    obs_.assign(obs_builder_->size(), 0.0f);
    policy_target_ = rest_targets();
    low_cmd_msg_.cmd_type = static_cast<int8_t>(cmd_type_);
    low_cmd_msg_.motor_cmd.resize(n);

    low_state_sub_ = create_subscription<LowState>(
      low_state_topic, rclcpp::SensorDataQoS(),
      [this](LowState::ConstSharedPtr msg) {on_low_state(*msg);});
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        cmd_vel_ = {msg->linear.x, msg->linear.y, msg->angular.z};
        last_cmd_vel_ = SteadyClock::now();
      });
    remote_sub_ = create_subscription<RemoteControllerState>(
      remote_topic, 10,
      [this](RemoteControllerState::ConstSharedPtr msg) {on_remote(*msg);});

    low_cmd_pub_ = create_publisher<LowCmd>(low_cmd_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    if (publish_debug) {
      obs_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("~/observation", 10);
      action_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("~/action", 10);
    }
    rpc_client_ = create_client<RpcService>(rpc_service);

    auto service = [this](const std::string & name, auto handler) {
        return create_service<std_srvs::srv::Trigger>(
          "~/" + name, [this, handler](
            const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            res->success = (this->*handler)(res->message);
          });
      };
    start_srv_ = service("start", &T1PpowalkPolicyNode::start);
    stop_srv_ = service("stop", &T1PpowalkPolicyNode::stop);
    release_srv_ = service("release", &T1PpowalkPolicyNode::release);
    home_srv_ = service("home", &T1PpowalkPolicyNode::home);
    custom_srv_ = service("enter_custom_mode", &T1PpowalkPolicyNode::enter_custom_mode);
    prepare_srv_ = service("exit_custom_mode", &T1PpowalkPolicyNode::exit_custom_mode);

    create_timers();
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {publish_status();});
    scan_models();
    models_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {scan_models();});
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return on_set_parameters(params);});

    RCLCPP_INFO(
      get_logger(), "t1_ppowalk ready: %zu joints, %zu actions, %zu obs, policy %.0f Hz, LowCmd "
      "%.0f Hz on %s (%s), start allowed below %.0f deg tilt. Model: %s", n, action_index_.size(),
      obs_builder_->size(), policy_rate_hz_, command_rate_hz_, low_cmd_topic.c_str(),
      enable_lowcmd_ ? "ENABLED" : "disabled", max_start_tilt_deg_,
      policy_path_.empty() ? "(none)" : policy_path_.c_str());
    RCLCPP_INFO(
      get_logger(), "Velocity command from %s; remote START %s, BACK %s (%s)",
      use_cmd_vel_ ? "cmd_vel" : use_remote_ ? "the remote controller" : "the GUI",
      remote_start_ ? "= Start" : "disabled", remote_home_ ? "= Home" : "disabled",
      remote_topic.c_str());
    if (!model_warnings_.empty()) {
      RCLCPP_WARN(get_logger(), "Model metadata: %s", model_warnings_.c_str());
    }
  }

private:
  enum class Mode { kIdle, kHoming, kHolding, kRamping, kRunning };
  static const char * mode_name(Mode m)
  {
    static const char * names[] = {"IDLE", "HOMING", "HOLDING", "RAMPING", "RUNNING"};
    return names[static_cast<int>(m)];
  }

  // ------------------------------------------------------------ validation

  std::string check_home(const std::vector<double> & home) const
  {
    return home.size() == joint_names_.size() ? "" : "home_pose must have len(joint_names) entries";
  }

  std::string check_motor_params(
    const std::vector<double> & kp, const std::vector<double> & kd,
    const std::vector<double> & lower, const std::vector<double> & upper) const
  {
    const size_t n = joint_names_.size();
    if (kp.size() != n || kd.size() != n) {
      return "kp and kd must have len(joint_names) entries";
    }
    if (lower.size() != n || upper.size() != n) {
      return "position_lower and position_upper must have len(joint_names) entries";
    }
    for (size_t i = 0; i < n; ++i) {
      if (kp[i] < 0.0 || kd[i] < 0.0) {
        return "kp and kd must be >= 0 (" + joint_names_[i] + ")";
      }
      if (lower[i] > upper[i]) {
        return "position_lower > position_upper for " + joint_names_[i];
      }
    }
    return "";
  }

  std::string check_names(const std::vector<std::string> & names, const std::string & what) const
  {
    for (const auto & name : names) {
      if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end()) {
        return what + ": unknown joint '" + name + "'";
      }
    }
    return "";
  }

  std::string check_command_joints(const std::vector<std::string> & names) const
  {
    std::string error = check_names(names, "command_joints");
    if (!error.empty()) {
      return error;
    }
    for (const auto & a : action_joint_names_) {
      if (std::find(names.begin(), names.end(), a) == names.end()) {
        return "command_joints must include every action joint (missing " + a + ")";
      }
    }
    for (const auto & a : always_commanded_) {
      if (std::find(names.begin(), names.end(), a) == names.end()) {
        return "command_joints must include " + a + " (always_commanded_joints)";
      }
    }
    return "";
  }

  static std::string check_remote(
    bool use_remote, bool use_cmd_vel, double deadzone, double timeout)
  {
    if (use_remote && use_cmd_vel) {
      return "use_remote and use_cmd_vel are exclusive (one velocity command source)";
    }
    if (deadzone < 0.0 || deadzone > 0.9) {
      return "remote_deadzone must be in [0, 0.9]";
    }
    if (timeout <= 0.0) {
      return "remote_timeout_s must be > 0";
    }
    return "";
  }

  // models_dir, or the folder of the given policy path if empty; "~" expanded.
  static std::string resolve_models_dir(std::string dir, const std::string & policy_path)
  {
    if (dir.empty()) {
      dir = policy_path.empty() ? "~/t1_ppowalk_models" :
        std::filesystem::path(policy_path).parent_path().string();
    }
    if (!dir.empty() && dir[0] == '~') {
      const char * home = std::getenv("HOME");
      dir = std::string(home ? home : "") + dir.substr(1);
    }
    dir = std::filesystem::path(dir).lexically_normal().string();
    while (dir.size() > 1 && dir.back() == '/') {
      dir.pop_back();
    }
    return dir;
  }

  // *.onnx files of models_dir, sorted (offered by the GUI's model dropdown).
  void scan_models()
  {
    std::vector<std::string> names;
    std::error_code ec;
    std::filesystem::directory_iterator it(models_dir_, ec);
    models_error_ = ec ? ec.message() : "";
    for (; !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
      std::error_code file_ec;
      const auto name = it->path().filename().string();
      if (it->path().extension() == ".onnx" && it->is_regular_file(file_ec) &&
        name.find_first_of("\"\\") == std::string::npos)
      {
        names.push_back(name);
      }
    }
    std::sort(names.begin(), names.end());
    models_ = std::move(names);
  }

  static std::string json_safe(const std::string & text)
  {
    std::string out;
    for (char ch : text) {
      if (ch != '"' && ch != '\\') {
        out += ch;
      }
    }
    return out;
  }

  std::vector<bool> name_mask(const std::vector<std::string> & names) const
  {
    std::vector<bool> mask(joint_names_.size(), false);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      mask[i] = std::find(names.begin(), names.end(), joint_names_[i]) != names.end();
    }
    return mask;
  }

  void apply_command_joints(const std::vector<std::string> & names)
  {
    command_joint_names_ = names;
    commanded_ = name_mask(names);
  }

  // Targets with zero action: action joints at the default pose, the rest at
  // their hold targets.
  std::vector<double> rest_targets() const
  {
    std::vector<double> t(joint_names_.size());
    for (size_t i = 0; i < t.size(); ++i) {
      t[i] = is_action_[i] ? default_joint_pos_[i] : held_targets_[i];
    }
    return t;
  }

  bool running() const
  {
    return mode_ == Mode::kRamping || mode_ == Mode::kRunning;
  }

  // Motors with error flags: the live ones, or those latched during this run.
  std::vector<bool> offline_mask() const
  {
    if (!detect_offline_) {
      return std::vector<bool>(joint_names_.size(), false);
    }
    return running() ? session_offline_ : offline_now_;
  }

  // Joints whose observation is reported at the hold pose, at rest.
  std::vector<bool> pinned_mask() const
  {
    const auto offline = offline_mask();
    std::vector<bool> pin(joint_names_.size());
    for (size_t i = 0; i < pin.size(); ++i) {
      pin[i] = manual_pin_[i] || !commanded_[i] || offline[i];
    }
    return pin;
  }

  std::vector<std::string> names_where(const std::vector<bool> & mask) const
  {
    std::vector<std::string> out;
    for (size_t i = 0; i < mask.size(); ++i) {
      if (mask[i]) {
        out.push_back(joint_names_[i]);
      }
    }
    return out;
  }

  static std::string json_list(const std::vector<std::string> & names)
  {
    std::string out = "[";
    for (const auto & n : names) {
      out += (out.size() > 1 ? ",\"" : "\"") + n + "\"";
    }
    return out + "]";
  }

  std::vector<double> expand_action_scale(std::vector<double> scale, std::string & error) const
  {
    if (scale.size() == 1) {
      scale.assign(action_index_.size(), scale[0]);
    }
    if (scale.size() != action_index_.size()) {
      error = "action_scale must have 1 or len(action_joint_names) entries";
    }
    return scale;
  }

  std::unique_ptr<ObservationBuilder> make_observation_builder(
    const std::vector<double> & default_joint_pos, const std::vector<double> & scales,
    std::string & error) const
  {
    if (default_joint_pos.size() != joint_names_.size()) {
      error = "default_joint_pos must have len(joint_names) entries";
      return nullptr;
    }
    try {
      return std::make_unique<ObservationBuilder>(
        observation_terms_, default_joint_pos, scales, action_index_.size());
    } catch (const std::exception & e) {
      error = e.what();
      return nullptr;
    }
  }

  std::unique_ptr<OnnxPolicy> load_policy(const std::string & path, std::string & error) const
  {
    std::unique_ptr<OnnxPolicy> policy;
    try {
      policy = std::make_unique<OnnxPolicy>(path, num_threads_);
    } catch (const std::exception & e) {
      error = "Cannot load policy " + path + ": " + e.what();
      return nullptr;
    }
    if (policy->num_obs() != obs_builder_->size() ||
      policy->num_actions() != action_index_.size())
    {
      error = "Policy is " + std::to_string(policy->num_obs()) + " -> " +
        std::to_string(policy->num_actions()) + ", expected " +
        std::to_string(obs_builder_->size()) + " -> " + std::to_string(action_index_.size());
      return nullptr;
    }
    return policy;
  }

  static std::vector<std::string> split_csv(const std::string & text)
  {
    std::vector<std::string> out;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
      out.push_back(item);
    }
    return out;
  }

  // Compares the mjlab export metadata (CSV strings, values rounded to 3
  // decimals) with the parameters. Empty when everything matches; a mismatch
  // does not block the model, it is reported in ~/status and the log.
  std::string check_model_metadata(const OnnxPolicy & policy) const
  {
    std::vector<std::string> issues;
    const auto names = split_csv(policy.metadata("joint_names"));
    if (names.empty()) {
      return "no mjlab metadata in the model (not checked)";
    }
    if (names != joint_names_) {
      issues.push_back("joint_names differ");
    }
    auto compare = [&](const std::string & key, const std::vector<double> & expected,
        double tol) {
        const auto values = split_csv(policy.metadata(key));
        if (values.empty()) {
          return;
        }
        if (values.size() != expected.size()) {
          issues.push_back(key + " has " + std::to_string(values.size()) + " entries, expected " +
            std::to_string(expected.size()));
          return;
        }
        for (size_t i = 0; i < values.size(); ++i) {
          double v = 0.0;
          try {
            v = std::stod(values[i]);
          } catch (const std::exception &) {
            issues.push_back(key + " is not numeric");
            return;
          }
          if (std::abs(v - expected[i]) > tol) {
            issues.push_back(key + "[" + std::to_string(i) + "] = " + values[i]);
            return;
          }
        }
      };
    compare("default_joint_pos", default_joint_pos_, 1e-3);
    compare("action_scale", action_scale_, 1e-3);
    compare("joint_stiffness", kp_, 2e-3);
    compare("joint_damping", kd_, 2e-3);
    std::string observation;
    for (const auto & t : observation_terms_) {
      observation += (observation.empty() ? "" : ",") + t;
    }
    const auto obs_names = policy.metadata("observation_names");
    if (!obs_names.empty() && obs_names != observation) {
      issues.push_back("observation_names = " + obs_names);
    }
    std::string out;
    for (const auto & s : issues) {
      out += (out.empty() ? "" : "; ") + s;
    }
    return out;
  }

  void create_timers()
  {
    policy_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / policy_rate_hz_), [this]() {policy_step();});
    command_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / command_rate_hz_), [this]() {publish_command();});
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    static const std::set<std::string> kReadOnly = {
      "joint_names", "action_joint_names", "observation_terms", "num_obs", "num_threads",
      "low_state_topic", "low_cmd_topic", "cmd_vel_topic", "remote_topic", "rpc_service",
      "publish_debug", "always_commanded_joints", "cmd_type", "effort_limit"};
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    auto action_scale = action_scale_;
    auto default_joint_pos = default_joint_pos_;
    auto observation_scales = observation_scales_;
    auto home_pose = home_pose_;
    auto kp = kp_, kd = kd_, lower = position_lower_, upper = position_upper_;
    double damping_kd = damping_kd_, max_target_delta = max_target_delta_;
    bool enable_lowcmd = enable_lowcmd_;
    int64_t custom_mode = custom_mode_, prepare_mode = prepare_mode_;
    auto cmd = cmd_;
    auto cmd_min = cmd_min_;
    auto cmd_max = cmd_max_;
    auto cmd_offset = cmd_offset_;
    bool offsets_when_standing = offsets_when_standing_;
    double action_clip = action_clip_, data_timeout = data_timeout_;
    double policy_rate = policy_rate_hz_, command_rate = command_rate_hz_;
    double max_tilt = max_start_tilt_deg_, ramp = start_ramp_s_;
    double home_vel = home_max_vel_, home_min = home_min_duration_;
    double cmd_vel_timeout = cmd_vel_timeout_;
    bool use_cmd_vel = use_cmd_vel_;
    bool use_remote = use_remote_, remote_start = remote_start_, remote_home = remote_home_;
    double remote_deadzone = remote_deadzone_, remote_timeout = remote_timeout_;
    auto command_joints = command_joint_names_;
    auto held_targets = held_targets_;
    auto pinned_names = pinned_names_;
    bool detect_offline = detect_offline_;
    bool command_joints_changed = false;
    bool motor_params_changed = false;
    std::optional<std::string> policy_path;
    std::optional<std::string> models_dir;
    bool rebuild_obs = false;
    try {
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (kReadOnly.count(name)) {
          result.reason = name + " is read-only (restart the node to change it)";
          return result;
        } else if (name == "action_scale") {
          action_scale = expand_action_scale(p.as_double_array(), result.reason);
        } else if (name == "default_joint_pos") {
          default_joint_pos = p.as_double_array();
          rebuild_obs = true;
        } else if (name == "observation_scales") {
          observation_scales = p.as_double_array();
          rebuild_obs = true;
        } else if (name == "home_pose") {
          home_pose = p.as_double_array();
          result.reason = check_home(home_pose);
        } else if (name == "kp") {
          kp = p.as_double_array();
          motor_params_changed = true;
        } else if (name == "kd") {
          kd = p.as_double_array();
          motor_params_changed = true;
        } else if (name == "position_lower") {
          lower = p.as_double_array();
          motor_params_changed = true;
        } else if (name == "position_upper") {
          upper = p.as_double_array();
          motor_params_changed = true;
        } else if (name == "damping_kd") {
          damping_kd = p.as_double();
        } else if (name == "max_target_delta") {
          max_target_delta = p.as_double();
        } else if (name == "enable_lowcmd") {
          enable_lowcmd = p.as_bool();
        } else if (name == "custom_mode") {
          custom_mode = p.as_int();
        } else if (name == "prepare_mode") {
          prepare_mode = p.as_int();
        } else if (name == "action_clip") {
          action_clip = p.as_double();
        } else if (name == "data_timeout_s") {
          data_timeout = p.as_double();
        } else if (name == "policy_rate_hz") {
          policy_rate = p.as_double();
        } else if (name == "command_rate_hz") {
          command_rate = p.as_double();
        } else if (name == "max_start_tilt_deg") {
          max_tilt = p.as_double();
        } else if (name == "start_ramp_s") {
          ramp = p.as_double();
        } else if (name == "home_max_vel") {
          home_vel = p.as_double();
        } else if (name == "home_min_duration_s") {
          home_min = p.as_double();
        } else if (name == "cmd_vx") {
          cmd[0] = p.as_double();
        } else if (name == "cmd_vy") {
          cmd[1] = p.as_double();
        } else if (name == "cmd_wz") {
          cmd[2] = p.as_double();
        } else if (name == "lin_vel_x_min") {
          cmd_min[0] = p.as_double();
        } else if (name == "lin_vel_y_min") {
          cmd_min[1] = p.as_double();
        } else if (name == "ang_vel_z_min") {
          cmd_min[2] = p.as_double();
        } else if (name == "lin_vel_x_max") {
          cmd_max[0] = p.as_double();
        } else if (name == "lin_vel_y_max") {
          cmd_max[1] = p.as_double();
        } else if (name == "ang_vel_z_max") {
          cmd_max[2] = p.as_double();
        } else if (name == "cmd_vx_offset") {
          cmd_offset[0] = p.as_double();
        } else if (name == "cmd_vy_offset") {
          cmd_offset[1] = p.as_double();
        } else if (name == "cmd_wz_offset") {
          cmd_offset[2] = p.as_double();
        } else if (name == "offsets_when_standing") {
          offsets_when_standing = p.as_bool();
        } else if (name == "use_cmd_vel") {
          use_cmd_vel = p.as_bool();
        } else if (name == "cmd_vel_timeout_s") {
          cmd_vel_timeout = p.as_double();
        } else if (name == "use_remote") {
          use_remote = p.as_bool();
        } else if (name == "remote_start") {
          remote_start = p.as_bool();
        } else if (name == "remote_home") {
          remote_home = p.as_bool();
        } else if (name == "remote_deadzone") {
          remote_deadzone = p.as_double();
        } else if (name == "remote_timeout_s") {
          remote_timeout = p.as_double();
        } else if (name == "policy_path") {
          policy_path = p.as_string();
        } else if (name == "models_dir") {
          models_dir = p.as_string();
        } else if (name == "command_joints") {
          command_joints = p.as_string_array();
          result.reason = check_command_joints(command_joints);
          command_joints_changed = command_joints != command_joint_names_;
        } else if (name == "held_joint_targets") {
          held_targets = p.as_double_array();
          if (held_targets.size() != joint_names_.size()) {
            result.reason = "held_joint_targets must have len(joint_names) entries";
          }
        } else if (name == "obs_pinned_joints") {
          pinned_names = p.as_string_array();
          result.reason = check_names(pinned_names, "obs_pinned_joints");
        } else if (name == "detect_offline") {
          detect_offline = p.as_bool();
        }
        if (!result.reason.empty()) {
          return result;
        }
      }
    } catch (const rclcpp::ParameterTypeException & e) {
      result.reason = e.what();
      return result;
    }
    if (motor_params_changed) {
      result.reason = check_motor_params(kp, kd, lower, upper);
      if (!result.reason.empty()) {
        return result;
      }
    }
    if (action_clip < 0.0 || data_timeout <= 0.0 || ramp < 0.0 || home_min < 0.0 ||
      home_vel <= 0.0 || cmd_vel_timeout <= 0.0 || damping_kd < 0.0 || max_target_delta < 0.0)
    {
      result.reason = "action_clip, start_ramp_s, home_min_duration_s, damping_kd, "
        "max_target_delta >= 0; data_timeout_s, home_max_vel, cmd_vel_timeout_s > 0";
      return result;
    }
    if (max_tilt <= 0.0 || max_tilt > 90.0) {
      result.reason = "max_start_tilt_deg must be in (0, 90]";
      return result;
    }
    result.reason = check_remote(use_remote, use_cmd_vel, remote_deadzone, remote_timeout);
    if (!result.reason.empty()) {
      return result;
    }
    if (policy_rate <= 0.0 || command_rate <= 0.0 || policy_rate > 1000.0 ||
      command_rate > 2000.0)
    {
      result.reason = "policy_rate_hz must be in (0, 1000] and command_rate_hz in (0, 2000]";
      return result;
    }
    for (int k = 0; k < 3; ++k) {
      if (cmd_min[k] > 0.0 || cmd_max[k] < 0.0) {
        result.reason = "velocity limits must include 0 (min <= 0 <= max)";
        return result;
      }
    }
    if (command_joints_changed && running()) {
      result.reason = "command_joints can only be changed while the policy is stopped";
      return result;
    }
    std::unique_ptr<ObservationBuilder> obs_builder;
    if (rebuild_obs) {
      obs_builder = make_observation_builder(default_joint_pos, observation_scales, result.reason);
      if (!obs_builder) {
        return result;
      }
    }
    std::unique_ptr<OnnxPolicy> policy;
    if (policy_path && *policy_path != policy_path_) {
      if (running()) {
        result.reason = "policy_path can only be changed while the policy is stopped";
        return result;
      }
      policy = load_policy(*policy_path, result.reason);
      if (!policy) {
        return result;
      }
    }

    action_scale_ = std::move(action_scale);
    home_pose_ = std::move(home_pose);
    kp_ = std::move(kp);
    kd_ = std::move(kd);
    position_lower_ = std::move(lower);
    position_upper_ = std::move(upper);
    damping_kd_ = damping_kd;
    max_target_delta_ = max_target_delta;
    if (enable_lowcmd != enable_lowcmd_) {
      RCLCPP_WARN(get_logger(), "LowCmd output %s", enable_lowcmd ? "ENABLED" : "disabled");
    }
    enable_lowcmd_ = enable_lowcmd;
    custom_mode_ = custom_mode;
    prepare_mode_ = prepare_mode;
    cmd_ = cmd;
    cmd_min_ = cmd_min;
    cmd_max_ = cmd_max;
    cmd_offset_ = cmd_offset;
    offsets_when_standing_ = offsets_when_standing;
    action_clip_ = action_clip;
    data_timeout_ = data_timeout;
    max_start_tilt_deg_ = max_tilt;
    start_ramp_s_ = ramp;
    home_max_vel_ = home_vel;
    home_min_duration_ = home_min;
    use_cmd_vel_ = use_cmd_vel;
    cmd_vel_timeout_ = cmd_vel_timeout;
    if (use_remote != use_remote_ || remote_start != remote_start_ || remote_home != remote_home_) {
      RCLCPP_WARN(
        get_logger(), "Remote controller: velocity command %s, START button %s, BACK button %s",
        use_remote ? "ON" : "off", remote_start ? "ON" : "off", remote_home ? "ON" : "off");
    }
    use_remote_ = use_remote;
    remote_start_ = remote_start;
    remote_home_ = remote_home;
    remote_deadzone_ = remote_deadzone;
    remote_timeout_ = remote_timeout;
    held_targets_ = std::move(held_targets);
    pinned_names_ = std::move(pinned_names);
    manual_pin_ = name_mask(pinned_names_);
    detect_offline_ = detect_offline;
    if (command_joints_changed) {
      apply_command_joints(command_joints);
      RCLCPP_WARN(get_logger(), "Now commanding %zu joints", command_joint_names_.size());
    }
    if (obs_builder) {
      default_joint_pos_ = std::move(default_joint_pos);
      observation_scales_ = std::move(observation_scales);
      obs_builder_ = std::move(obs_builder);
    }
    if (policy) {
      policy_ = std::move(policy);
      policy_path_ = *policy_path;
      RCLCPP_WARN(get_logger(), "Loaded policy %s", policy_path_.c_str());
    }
    if (policy_ && (policy || rebuild_obs || motor_params_changed ||
      std::find_if(params.begin(), params.end(), [](const rclcpp::Parameter & p) {
        return p.get_name() == "action_scale";
      }) != params.end()))
    {
      model_warnings_ = check_model_metadata(*policy_);
      if (!model_warnings_.empty()) {
        RCLCPP_WARN(get_logger(), "Model metadata: %s", model_warnings_.c_str());
      }
    }
    if (models_dir) {
      models_dir_param_ = *models_dir;
      models_dir_ = resolve_models_dir(models_dir_param_, policy_path_);
      scan_models();
    }
    if (policy_rate != policy_rate_hz_ || command_rate != command_rate_hz_) {
      policy_rate_hz_ = policy_rate;
      command_rate_hz_ = command_rate;
      create_timers();
    }
    result.successful = true;
    return result;
  }

  // ---------------------------------------------------------------- inputs

  void on_low_state(const LowState & msg)
  {
    const auto & imu = msg.imu_state;
    state_.base_ang_vel = {imu.gyro[0], imu.gyro[1], imu.gyro[2]};
    state_.base_quat = quat_from_rpy(imu.rpy[0], imu.rpy[1], imu.rpy[2]);
    const auto & q = state_.base_quat;
    const auto g = projected_gravity(q[0], q[1], q[2], q[3]);
    tilt_deg_ = tilt_deg(g[0], g[1], g[2]);
    last_imu_ = SteadyClock::now();

    const auto & motors =
      cmd_type_ == LowCmd::CMD_TYPE_SERIAL ? msg.motor_state_serial : msg.motor_state_parallel;
    const size_t n = joint_names_.size();
    if (motors.size() < n) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "LowState has %zu %s motor states, expected %zu",
        motors.size(), cmd_type_ == LowCmd::CMD_TYPE_SERIAL ? "serial" : "parallel", n);
      return;
    }
    const bool session = running();
    for (size_t i = 0; i < n; ++i) {
      state_.joint_pos[i] = motors[i].q;
      state_.joint_vel[i] = motors[i].dq;
      // reserve[0] holds the motor error flags (0 = healthy).
      offline_now_[i] = motors[i].reserve[0] != 0;
      if (session && offline_now_[i]) {
        session_offline_[i] = true;
      }
    }
    last_js_ = SteadyClock::now();
  }

  // Booster remote controller. Buttons act on the press (rising edge) only; a
  // button already held when the first message arrives does not count.
  void on_remote(const RemoteControllerState & msg)
  {
    remote_axes_ = {msg.lx, msg.ly, msg.rx, msg.ry};
    const bool buttons[kNumRemoteButtons] = {
      msg.a, msg.b, msg.x, msg.y, msg.lb, msg.rb, msg.lt, msg.rt, msg.ls, msg.rs, msg.back,
      msg.start};
    remote_buttons_ = 0;
    for (int b = 0; b < kNumRemoteButtons; ++b) {
      if (buttons[b]) {
        remote_buttons_ |= 1u << b;
      }
    }
    last_remote_ = SteadyClock::now();
    const bool start_pressed = remote_start_down_ && !*remote_start_down_ && msg.start;
    const bool back_pressed = remote_back_down_ && !*remote_back_down_ && msg.back;
    remote_start_down_ = msg.start;
    remote_back_down_ = msg.back;
    if (back_pressed) {
      if (!remote_home_) {
        remote_event("BACK button ignored (remote_home is off)");
      } else {
        RCLCPP_INFO(get_logger(), "Remote: BACK button pressed");
        std::string message;
        const bool ok = home(message);
        remote_event(ok ? "BACK button: " + message : "BACK button: Home refused (" + message + ")");
      }
    }
    if (start_pressed) {
      if (!remote_start_) {
        remote_event("START button ignored (remote_start is off)");
      } else {
        RCLCPP_INFO(get_logger(), "Remote: START button pressed");
        std::string message;
        const bool ok = start(message);
        remote_event(
          ok ? "START button: policy started" : "START button: refused (" + message + ")");
      }
    }
  }

  void remote_event(const std::string & text)
  {
    remote_event_ = json_safe(text);
    last_remote_event_ = SteadyClock::now();
    ++remote_events_;
  }

  static double age(const std::optional<SteadyClock::time_point> & t)
  {
    return t ? std::chrono::duration<double>(SteadyClock::now() - *t).count() : -1.0;
  }

  bool remote_fresh() const
  {
    return last_remote_ && age(last_remote_) < remote_timeout_;
  }

  bool data_fresh() const
  {
    return last_imu_ && last_js_ && age(last_imu_) < data_timeout_ &&
           age(last_js_) < data_timeout_;
  }

  bool upright() const
  {
    return last_imu_ && age(last_imu_) < data_timeout_ && tilt_deg_ <= max_start_tilt_deg_;
  }

  // Requested command: cmd_vel or the remote sticks (0 while their data is
  // stale), else the GUI parameters.
  std::array<double, 3> raw_command() const
  {
    if (use_cmd_vel_) {
      return (last_cmd_vel_ && age(last_cmd_vel_) < cmd_vel_timeout_) ?
             cmd_vel_ : std::array<double, 3>{0.0, 0.0, 0.0};
    }
    if (use_remote_) {
      if (!remote_fresh()) {
        return {0.0, 0.0, 0.0};
      }
      // Booster sticks: push front = -1 (ly), push right = +1 (lx, rx). Stick
      // front = forward (+vx), stick right = -y (robot y points left) and a
      // clockwise turn (-wz).
      const double lx = remote_axes_[0], ly = remote_axes_[1], rx = remote_axes_[2];
      return {stick_command(-ly, remote_deadzone_, cmd_min_[0], cmd_max_[0]),
        stick_command(-lx, remote_deadzone_, cmd_min_[1], cmd_max_[1]),
        stick_command(-rx, remote_deadzone_, cmd_min_[2], cmd_max_[2])};
    }
    return cmd_;
  }

  std::string command_source() const
  {
    if (use_cmd_vel_) {
      return last_cmd_vel_ && age(last_cmd_vel_) < cmd_vel_timeout_ ? "cmd_vel" :
             "cmd_vel (stale: 0)";
    }
    if (use_remote_) {
      return remote_fresh() ? "remote" : "remote (no data: 0)";
    }
    return "gui";
  }

  // Command fed to the policy: requested + calibration offsets (only while
  // motion is requested, unless offsets_when_standing), clamped to the limits.
  std::array<double, 3> velocity_command() const
  {
    std::array<double, 3> c = raw_command();
    const bool standing = c[0] == 0.0 && c[1] == 0.0 && c[2] == 0.0;
    for (int k = 0; k < 3; ++k) {
      if (!standing || offsets_when_standing_) {
        c[k] += cmd_offset_[k];
      }
      c[k] = std::clamp(c[k], cmd_min_[k], cmd_max_[k]);
    }
    return c;
  }

  // -------------------------------------------------------------- services

  std::vector<double> current_pose() const
  {
    return target_ ? *target_ : state_.joint_pos;
  }

  bool home(std::string & message)
  {
    if (!data_fresh()) {
      message = "no fresh LowState data";
      return false;
    }
    move_from_ = current_pose();
    move_duration_ = safe_move_duration(move_from_, home_pose_, home_max_vel_, home_min_duration_);
    move_start_ = SteadyClock::now();
    if (running()) {
      RCLCPP_WARN(get_logger(), "Policy stopped: moving home");
    }
    mode_ = Mode::kHoming;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "moving home (%.1f s)", move_duration_);
    message = buf;
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  bool start(std::string & message)
  {
    std::string legs;
    if (detect_offline_) {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (is_leg_[i] && offline_now_[i]) {
          legs += (legs.empty() ? "" : ", ") + joint_names_[i];
        }
      }
    }
    if (!policy_) {
      message = "no policy loaded (set policy_path)";
    } else if (running()) {
      message = "already running";
    } else if (mode_ == Mode::kHoming) {
      message = "wait until homing is finished";
    } else if (!data_fresh()) {
      message = "no fresh LowState data";
    } else if (!legs.empty()) {
      message = "leg motor error: " + legs;
    } else if (!upright()) {
      char buf[96];
      std::snprintf(
        buf, sizeof(buf), "robot not upright: tilt %.1f deg > %.1f deg", tilt_deg_,
        max_start_tilt_deg_);
      message = buf;
    } else {
      std::fill(action_.begin(), action_.end(), 0.0f);
      policy_target_ = rest_targets();
      session_offline_ = offline_now_;
      move_from_ = current_pose();
      move_start_ = SteadyClock::now();
      mode_ = start_ramp_s_ > 0.0 ? Mode::kRamping : Mode::kRunning;
      step_ = 0;
      message = "started";
      RCLCPP_INFO(get_logger(), "Policy started (tilt %.1f deg)", tilt_deg_);
      return true;
    }
    RCLCPP_WARN(get_logger(), "Start refused: %s", message.c_str());
    return false;
  }

  // Stop = freeze: keep commanding the last target (the robot stays stiff).
  bool stop(std::string & message)
  {
    if (mode_ == Mode::kIdle) {
      message = "nothing to stop";
      return false;
    }
    if (target_) {
      mode_ = Mode::kHolding;
      message = "holding the current pose";
    } else {
      mode_ = Mode::kIdle;
      message = "stopped";
    }
    RCLCPP_WARN(get_logger(), "Stop: %s", message.c_str());
    return true;
  }

  // Release = damp every joint (the robot goes limp).
  bool release(std::string & message)
  {
    mode_ = Mode::kIdle;
    target_.reset();
    message = "released (damping)";
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return true;
  }

  bool change_mode(int64_t mode, const std::string & what, std::string & message)
  {
    if (!rpc_client_->service_is_ready()) {
      message = std::string("RPC service ") + rpc_client_->get_service_name() + " not available";
      RCLCPP_WARN(get_logger(), "%s: %s", what.c_str(), message.c_str());
      return false;
    }
    auto req = std::make_shared<RpcService::Request>();
    req->msg.api_id = kChangeModeApi;
    req->msg.body = "{\"mode\":" + std::to_string(mode) + "}";
    rpc_client_->async_send_request(
      req, [this, what](rclcpp::Client<RpcService>::SharedFuture future) {
        const auto resp = future.get();
        rpc_result_ = json_safe(
          what + ": status " + std::to_string(resp->msg.status) +
          (resp->msg.body.empty() ? "" : " " + resp->msg.body));
        RCLCPP_INFO(get_logger(), "%s", rpc_result_.c_str());
      });
    message = what + " requested (ChangeMode " + std::to_string(mode) + ")";
    rpc_result_ = json_safe(message);
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return true;
  }

  bool enter_custom_mode(std::string & message)
  {
    return change_mode(custom_mode_, "Custom mode", message);
  }

  bool exit_custom_mode(std::string & message)
  {
    if (running()) {
      message = "stop the policy first";
      return false;
    }
    return change_mode(prepare_mode_, "Prepare mode", message);
  }

  // ------------------------------------------------------------- execution

  void policy_step()
  {
    if (!running()) {
      return;
    }
    if (!data_fresh()) {
      // On-robot sensor loss: freeze instead of walking blind.
      RCLCPP_ERROR(get_logger(), "Sensor data timed out: holding the current pose");
      mode_ = target_ ? Mode::kHolding : Mode::kIdle;
      return;
    }
    state_.command = velocity_command();
    obs_state_ = state_;
    const auto pin = pinned_mask();
    for (size_t i = 0; i < pin.size(); ++i) {
      if (pin[i]) {
        obs_state_.joint_pos[i] = is_action_[i] ? default_joint_pos_[i] : held_targets_[i];
        obs_state_.joint_vel[i] = 0.0;
      }
    }
    const auto t0 = SteadyClock::now();
    obs_builder_->build(obs_state_, action_, obs_);
    const auto & out = policy_->infer(obs_);
    inference_ms_ = std::chrono::duration<double, std::milli>(SteadyClock::now() - t0).count();
    auto target = rest_targets();
    for (size_t k = 0; k < action_.size(); ++k) {
      float a = out[k];
      if (action_clip_ > 0.0) {
        a = std::clamp(a, static_cast<float>(-action_clip_), static_cast<float>(action_clip_));
      }
      action_[k] = a;
      target[action_index_[k]] += action_scale_[k] * a;
    }
    policy_target_ = std::move(target);
    ++step_;
    if (obs_pub_) {
      std_msgs::msg::Float32MultiArray m;
      m.data = obs_;
      obs_pub_->publish(m);
      m.data = action_;
      action_pub_->publish(m);
    }
  }

  void publish_command()
  {
    const auto now = SteadyClock::now();
    const double t = std::chrono::duration<double>(now - move_start_).count();
    switch (mode_) {
      case Mode::kIdle:
        break;
      case Mode::kHoming: {
          const double s = t / std::max(move_duration_, 1e-6);
          target_ = blend(move_from_, home_pose_, smooth_alpha(s));
          if (s >= 1.0) {
            mode_ = Mode::kHolding;
            RCLCPP_INFO(get_logger(), "Home reached: holding");
          }
          break;
        }
      case Mode::kHolding:
        break;
      case Mode::kRamping: {
          const double s = t / std::max(start_ramp_s_, 1e-6);
          target_ = blend(move_from_, policy_target_, smooth_alpha(s));
          if (s >= 1.0) {
            mode_ = Mode::kRunning;
          }
          break;
        }
      case Mode::kRunning:
        target_ = policy_target_;
        break;
    }
    if (!enable_lowcmd_) {
      return;
    }
    // Without fresh LowState, IDLE, for non-commanded joints and for motors
    // with error flags: damping at the measured position.
    const bool active = mode_ != Mode::kIdle && target_ && data_fresh();
    const auto offline = offline_mask();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto & m = low_cmd_msg_.motor_cmd[i];
      const double q = state_.joint_pos[i];
      m.dq = 0.0f;
      m.tau = 0.0f;
      m.weight = 1.0f;
      if (active && commanded_[i] && !offline[i]) {
        double target = std::clamp((*target_)[i], position_lower_[i], position_upper_[i]);
        if (max_target_delta_ > 0.0) {
          target = std::clamp(target, q - max_target_delta_, q + max_target_delta_);
        }
        m.q = static_cast<float>(target);
        m.kp = static_cast<float>(kp_[i]);
        m.kd = static_cast<float>(kd_[i]);
      } else {
        m.q = static_cast<float>(q);
        m.kp = 0.0f;
        m.kd = static_cast<float>(damping_kd_);
      }
    }
    low_cmd_pub_->publish(low_cmd_msg_);
    ++lowcmd_count_;
  }

  void publish_status()
  {
    const auto c = velocity_command();
    const std::string path = json_safe(policy_path_);
    char buf[1024];
    std::snprintf(
      buf, sizeof(buf),
      "{\"mode\":\"%s\",\"step\":%d,\"upright\":%s,\"tilt_deg\":%.2f,\"max_start_tilt_deg\":%.1f,"
      "\"data_fresh\":%s,\"imu_age\":%.3f,\"joint_states_age\":%.3f,\"inference_ms\":%.3f,"
      "\"policy_loaded\":%s,\"policy_path\":\"%s\",\"command\":[%.3f,%.3f,%.3f],"
      "\"command_source\":\"%s\"}",
      mode_name(mode_), step_, upright() ? "true" : "false", last_imu_ ? tilt_deg_ : -1.0,
      max_start_tilt_deg_, data_fresh() ? "true" : "false", age(last_imu_), age(last_js_),
      inference_ms_, policy_ ? "true" : "false", path.c_str(), c[0], c[1], c[2],
      command_source().c_str());
    std::string json = buf;
    json.pop_back();
    const auto raw = raw_command();
    char extra[256];
    std::snprintf(
      extra, sizeof(extra),
      ",\"command_raw\":[%.3f,%.3f,%.3f],\"command_offset\":[%.3f,%.3f,%.3f],"
      "\"enable_lowcmd\":%s,\"lowcmd_publishing\":%s,\"lowcmd_count\":%lu",
      raw[0], raw[1], raw[2], cmd_offset_[0], cmd_offset_[1], cmd_offset_[2],
      enable_lowcmd_ ? "true" : "false", enable_lowcmd_ && data_fresh() ? "true" : "false",
      static_cast<unsigned long>(lowcmd_count_));
    json += extra;
    json += ",\"command_joints\":" + json_list(command_joint_names_) +
      ",\"pinned_joints\":" + json_list(names_where(pinned_mask())) +
      ",\"offline_detected\":" + json_list(names_where(offline_now_)) +
      ",\"offline_joints\":" + json_list(names_where(offline_mask())) + ",\"remote\":" +
      remote_status() + ",\"models_dir\":\"" + json_safe(models_dir_) + "\",\"models\":" +
      json_list(models_) + ",\"models_error\":\"" + json_safe(models_error_) +
      "\",\"model_warnings\":\"" + json_safe(model_warnings_) + "\",\"rpc_result\":\"" +
      rpc_result_ + "\"}";
    std_msgs::msg::String msg;
    msg.data = json;
    status_pub_->publish(msg);
  }

  std::string remote_status() const
  {
    std::vector<std::string> buttons;
    for (int b = 0; b < kNumRemoteButtons; ++b) {
      if (remote_buttons_ & (1u << b)) {
        buttons.push_back(kRemoteButtonNames[b]);
      }
    }
    char buf[384];
    std::snprintf(
      buf, sizeof(buf),
      "{\"use_remote\":%s,\"remote_start\":%s,\"remote_home\":%s,\"fresh\":%s,\"age\":%.3f,"
      "\"sticks\":[%.3f,%.3f,%.3f,%.3f],\"events\":%d,\"event_age\":%.1f,\"last_event\":\"",
      use_remote_ ? "true" : "false", remote_start_ ? "true" : "false",
      remote_home_ ? "true" : "false", remote_fresh() ? "true" : "false", age(last_remote_),
      remote_axes_[0], remote_axes_[1], remote_axes_[2], remote_axes_[3], remote_events_,
      age(last_remote_event_));
    return buf + remote_event_ + "\",\"buttons\":" +
           json_list(remote_fresh() ? buttons : std::vector<std::string>{}) + "}";
  }

  // Parameters.
  std::string policy_path_;
  std::string models_dir_param_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> action_joint_names_;
  std::vector<size_t> action_index_;
  std::vector<double> default_joint_pos_;
  std::vector<std::string> observation_terms_;
  std::vector<double> observation_scales_;
  std::vector<double> action_scale_;
  double action_clip_{0.0};
  double policy_rate_hz_{50.0};
  double command_rate_hz_{200.0};
  int num_threads_{1};
  double data_timeout_{0.1};
  std::vector<double> kp_;
  std::vector<double> kd_;
  std::vector<double> effort_limit_;
  std::vector<double> position_lower_;
  std::vector<double> position_upper_;
  double damping_kd_{1.0};
  double max_target_delta_{0.0};
  bool enable_lowcmd_{false};
  int cmd_type_{LowCmd::CMD_TYPE_SERIAL};
  int64_t custom_mode_{3};
  int64_t prepare_mode_{1};
  std::array<double, 3> cmd_{0.0, 0.0, 0.0};
  std::array<double, 3> cmd_min_{-1.0, -1.0, -0.5};
  std::array<double, 3> cmd_max_{1.0, 1.0, 0.5};
  std::array<double, 3> cmd_offset_{0.0, 0.0, 0.0};
  bool offsets_when_standing_{false};
  bool use_cmd_vel_{false};
  double cmd_vel_timeout_{0.5};
  bool use_remote_{true};
  bool remote_start_{true};
  bool remote_home_{true};
  double remote_deadzone_{0.1};
  double remote_timeout_{0.5};
  double max_start_tilt_deg_{30.0};
  double start_ramp_s_{1.0};
  std::vector<double> home_pose_;
  std::vector<std::string> command_joint_names_;
  std::vector<std::string> always_commanded_;
  std::vector<double> held_targets_;
  std::vector<std::string> pinned_names_;
  bool detect_offline_{true};
  double home_max_vel_{0.3};
  double home_min_duration_{2.0};

  // State.
  std::string models_dir_;  // resolved models_dir
  std::vector<std::string> models_;  // *.onnx in models_dir_
  std::string models_error_;
  std::string model_warnings_;  // metadata mismatches of the loaded model
  std::string rpc_result_;  // last ChangeMode request / response
  std::unique_ptr<ObservationBuilder> obs_builder_;
  std::unique_ptr<OnnxPolicy> policy_;
  std::vector<float> obs_;
  std::vector<float> action_;
  std::vector<double> policy_target_;
  std::optional<std::vector<double>> target_;  // last commanded target
  std::vector<double> move_from_;
  SteadyClock::time_point move_start_{};
  double move_duration_{0.0};
  int step_{0};
  double inference_ms_{0.0};
  double tilt_deg_{180.0};
  Mode mode_{Mode::kIdle};
  RobotState state_;
  RobotState obs_state_;  // state_ with pinned joints replaced
  std::vector<bool> is_action_;
  std::vector<bool> is_leg_;
  std::vector<bool> commanded_;
  std::vector<bool> manual_pin_;
  std::vector<bool> offline_now_;
  std::vector<bool> session_offline_;
  std::optional<SteadyClock::time_point> last_imu_;
  std::optional<SteadyClock::time_point> last_js_;
  std::array<double, 3> cmd_vel_{0.0, 0.0, 0.0};
  std::optional<SteadyClock::time_point> last_cmd_vel_;
  std::array<double, 4> remote_axes_{0.0, 0.0, 0.0, 0.0};  // lx, ly, rx, ry
  uint32_t remote_buttons_{0};
  std::optional<SteadyClock::time_point> last_remote_;
  std::optional<bool> remote_start_down_;  // START state in the last message
  std::optional<bool> remote_back_down_;  // BACK state in the last message
  std::string remote_event_;
  std::optional<SteadyClock::time_point> last_remote_event_;
  int remote_events_{0};
  uint64_t lowcmd_count_{0};

  // ROS interfaces.
  LowCmd low_cmd_msg_;
  rclcpp::Subscription<LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<RemoteControllerState>::SharedPtr remote_sub_;
  rclcpp::Publisher<LowCmd>::SharedPtr low_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr action_pub_;
  rclcpp::Client<RpcService>::SharedPtr rpc_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr custom_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_srv_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr policy_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr models_timer_;
};

}  // namespace t1_ppowalk

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<t1_ppowalk::T1PpowalkPolicyNode>());
  rclcpp::shutdown();
  return 0;
}
