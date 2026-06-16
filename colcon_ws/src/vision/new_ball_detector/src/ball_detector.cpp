#include "new_ball_detector/ball_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

static constexpr double head_x = 0.0;
static constexpr double head_y = 0.0;
static constexpr double MIN_ANG_SEP = 9.0 * M_PI / 180.0;

cv::Mat toCVMat(const sensor_msgs::msg::Image & src)
{
    if (src.encoding == sensor_msgs::image_encodings::BGR8) {
        return cv::Mat(src.height, src.width, CV_8UC3,
                const_cast<unsigned char *>(src.data.data()),
                src.step).clone();
    }
    if (src.encoding == sensor_msgs::image_encodings::RGB8) {
        cv::Mat rgb(src.height, src.width, CV_8UC3,
                const_cast<unsigned char *>(src.data.data()),
                src.step);
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    return cv_bridge::toCvCopy(
            std::make_shared<sensor_msgs::msg::Image>(src), "bgr8")->image;
}

BallDetectorNode::BallDetectorNode()
    : Node("ball_detector")
{
    RCLCPP_INFO(this->get_logger(), "Initializing BallDetectorNode (C++)");

    // --- PARAMS ---
    // Camera
    this->declare_parameter("camera_topic", "/boostercamera/head/rgb");
    this->declare_parameter("hfov", 93.0);
    this->declare_parameter("vfov_rad", 101.0);
    this->declare_parameter("head_z", 0.87);

    // Yolo
    this->declare_parameter("ball_radius", 0.11);
    this->declare_parameter("show_debug", false);
    this->declare_parameter("model_path", "");
    this->declare_parameter(
            "class_names",
            std::vector<std::string>{"ball", "goal", "robot", "L", "T", "X", "center"});
    this->declare_parameter("ball_map_topic", "/vision/map_ball");

    this->declare_parameter("proc_interval", 0.1);

    // tf2
    this->declare_parameter("base_frame", "pumas_base_link");
    this->declare_parameter("map_frame", "pumas_map");

    this->get_parameter("camera_topic", camera_topic_);
    this->get_parameter("hfov", hfov_deg);
    this->get_parameter("vfov_rad", vfov_deg);
    this->get_parameter("head_z", head_z_);

    this->get_parameter("ball_radius", ball_radius_);
    this->get_parameter("show_debug", show_debug_);

    this->get_parameter("proc_interval", proc_interval_);
    this->get_parameter("model_path", model_path_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("ball_map_topic", ball_map_topic_);
    std::vector<std::string> class_names;
    this->get_parameter("class_names", class_names);

    hfov_rad = (hfov_deg * M_PI) / 180.0;
    vfov_rad = (vfov_deg * M_PI) / 180.0;

    if (model_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "model_path parameter is empty!");
        throw std::runtime_error("model_path missing");
    }

    detector_ = std::make_unique<YoloDetector>(model_path_, class_names);

    // --- SUBSCRIBERS ---
    // Camera image
    sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, 1,
            //"/camera/color/image_raw", 1,
            //"/boostercamera/head/rgb", 1,
            std::bind(&BallDetectorNode::imageCallback, this, std::placeholders::_1));

    // --- PUBLISHERS ---
    // Ball position in the image frame
    pub_ball_ = this->create_publisher<pumas_vision_msgs::msg::VisionObject>(
            "/vision/ball", 1);

    // Ball position relative to the robot
    pub_ball_map_ = this->create_publisher<geometry_msgs::msg::Pose2D>(
            ball_map_topic_, 10);

    // Landmarks publisher for localization
    pub_landmarks_ = this->create_publisher<localization_msg::msg::VisionLandmarkArray>(
            "/vision/landmarks", 1);

    // Model preview (to view through rqt_image_view)
    pub_debug_img_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/vision/debug_image", 1);

    // --- SERVICES ---
    joint_client_ = this->create_client<joint_states_package::srv::HeadJoints>(
            "get_head_joints");
    while (!joint_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            throw std::runtime_error("Interrupted while waiting for head joints service");
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for head joints service...");
    }

    joint_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BallDetectorNode::jointStateRequest, this));

    last_proc_time_ = this->now();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    RCLCPP_INFO(
            this->get_logger(),
            "Ball topics: /vision/ball (%s), %s (map Pose2D)",
            base_frame_.c_str(), ball_map_topic_.c_str());
}

void BallDetectorNode::publishBallInMap(double bx, double by, const rclcpp::Time & stamp)
{
    geometry_msgs::msg::Pose2D pose_map;
    pose_map.theta = 0.0;

    geometry_msgs::msg::PointStamped pt_base;
    pt_base.header.stamp = stamp;
    pt_base.header.frame_id = base_frame_;
    pt_base.point.x = bx;
    pt_base.point.y = by;
    pt_base.point.z = 0.0;

    try {
        geometry_msgs::msg::PointStamped pt_map;
        tf_buffer_->transform(
                pt_base, pt_map, map_frame_,
                tf2::durationFromSec(0.1));
        pose_map.x = pt_map.point.x;
        pose_map.y = pt_map.point.y;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "TF %s -> %s failed (%s); assuming robot at map origin",
                base_frame_.c_str(), map_frame_.c_str(), ex.what());
        pose_map.x = bx;
        pose_map.y = by;
    }

    pub_ball_map_->publish(pose_map);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
}

std::vector<Detection> BallDetectorNode::angularPrune(
        std::vector<Detection> & dets, int img_width, double min_sep_rad)
{
    if (dets.empty()) {
        return {};
    }

    struct Candidate {
        Detection det;
        double angle;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(dets.size());
    for (auto & d : dets) {
        candidates.push_back({d, computeAngle(d.center_x, img_width)});
    }
    std::sort(
            candidates.begin(), candidates.end(),
            [](const Candidate & a, const Candidate & b) {return a.angle < b.angle;});

    std::vector<Candidate> filtered = {candidates[0]};
    for (size_t i = 1; i < candidates.size(); ++i) {
        double diff = std::atan2(
                std::sin(candidates[i].angle - filtered.back().angle),
                std::cos(candidates[i].angle - filtered.back().angle));
        if (std::abs(diff) > min_sep_rad) {
            filtered.push_back(candidates[i]);
        } else if (candidates[i].det.confidence > filtered.back().det.confidence) {
            filtered.back() = candidates[i];
        }
    }

    std::vector<Detection> result;
    for (auto & c : filtered) {
        result.push_back(c.det);
    }
    return result;
}

// double BallDetectorNode::computeAngle(double img_x, int img_width)
// {
//     double norm_x = (img_x - img_width / 2.0) / (img_width / 2.0);
//     double alpha_cam = -(norm_x * (hfov_rad / 2.0));
//     double alpha_body = alpha_cam + current_head_pan_;
//     return std::atan2(std::sin(alpha_body), std::cos(alpha_body));
// }
double BallDetectorNode::computeAngle(double img_x, int img_width)
{
    double f_x = (img_width / 2.0) / std::tan(hfov_rad / 2.0);

    double delta_x = (img_width / 2.0) - img_x; 
    double alpha_cam = std::atan2(delta_x, f_x);

    double alpha_body = alpha_cam + current_head_pan_;

    return std::atan2(std::sin(alpha_body), std::cos(alpha_body));
}
void BallDetectorNode::jointStateRequest()
{
    if (!joint_client_->service_is_ready()) {
        return;
    }

    auto request = std::make_shared<joint_states_package::srv::HeadJoints::Request>();

    joint_client_->async_send_request(
        request,
        [this](rclcpp::Client<joint_states_package::srv::HeadJoints>::SharedFuture result_future) {
            try {
                auto response = result_future.get();
                if (response->success) {
                    current_head_pan_ = response->pan;
                    current_head_tilt_ = response->tilt;
                RCLCPP_DEBUG(this->get_logger(), "Head joints: pan=%.3f, tilt=%.3f",
                    response->pan, response->tilt);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Head joint service returned failure");
                }
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "Joint service call failed: %s", e.what());
            }
        });
}

void BallDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        auto now = this->now();
        if ((now - last_proc_time_).seconds() < proc_interval_) {
            return;
        }
        last_proc_time_ = now;

        cv::Mat bgr;
        try {
            bgr = toCVMat(*msg);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Image conversion error: %s", e.what());
            return;
        }

        if (bgr.empty()) {
            return;
        }

        std::vector<Detection> detections = detector_->detect(bgr);

        for (auto & d : detections) {
            d.center_x *= bgr.cols;
            d.center_y *= bgr.rows;
            d.width *= bgr.cols;
            d.height *= bgr.rows;
        }

        for (const auto & d : detections) {
            if (d.class_name == "ball" && d.confidence > 0.60f) {
                auto [bx, by] = getBallPosition(d.center_x, d.center_y, bgr.cols, bgr.rows);
                const rclcpp::Time stamp = this->now();

                pumas_vision_msgs::msg::VisionObject obj_msg;
                obj_msg.header.stamp = stamp;
                obj_msg.header.frame_id = base_frame_;
                obj_msg.id = d.class_name;
                obj_msg.confidence = d.confidence;
                obj_msg.x = static_cast<int>(d.center_x);
                obj_msg.y = static_cast<int>(d.center_y);
                obj_msg.width = static_cast<int>(d.width);
                obj_msg.height = static_cast<int>(d.height);
                obj_msg.pose.position.x = bx;
                obj_msg.pose.position.y = by;
                obj_msg.pose.position.z = 0.0;
                pub_ball_->publish(obj_msg);

                publishBallInMap(bx, by, stamp);
            }
        }

        std::map<std::string, std::vector<Detection>> by_class;
        for (auto & d : detections) {
            by_class[d.class_name].push_back(d);
        }

        localization_msg::msg::VisionLandmarkArray lm_array;
        bool has_landmarks = false;

        for (auto & [cls, dets] : by_class) {
            auto pruned = angularPrune(dets, bgr.cols, MIN_ANG_SEP);
            for (const auto & d : pruned) {
                localization_msg::msg::VisionLandmark lm;
                static const std::vector<std::string> CLASS_NAMES =
                {"ball", "goal", "robot", "L", "T", "X", "center"};
                auto it = std::find(CLASS_NAMES.begin(), CLASS_NAMES.end(), d.class_name);
                lm.id = (it != CLASS_NAMES.end()) ?
                    static_cast<int>(std::distance(CLASS_NAMES.begin(), it)) : -1;
                lm.angle = computeAngle(d.center_x, bgr.cols);
                lm.confidence = d.confidence;
                lm_array.landmarks.push_back(lm);
                has_landmarks = true;
            }
        }

        if (has_landmarks) {
            pub_landmarks_->publish(lm_array);
        }

        if (pub_debug_img_->get_subscription_count() > 0) {
            for (const auto & d : detections) {
                double x1 = d.center_x - d.width / 2.0;
                double y1 = d.center_y - d.height / 2.0;
                double x2 = d.center_x + d.width / 2.0;
                double y2 = d.center_y + d.height / 2.0;

                cv::rectangle(
                        bgr, cv::Point(x1, y1), cv::Point(x2, y2),
                        cv::Scalar(0, 255, 0), 2);

                std::string label = d.class_name + " " + std::to_string(d.confidence).substr(0, 4);
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(
                        label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                cv::rectangle(
                        bgr,
                        cv::Point(x1, y1 - text_size.height - 5),
                        cv::Point(x1 + text_size.width, y1),
                        cv::Scalar(0, 255, 0), cv::FILLED);
                cv::putText(
                        bgr, label, cv::Point(x1, y1 - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            }

            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", bgr).toImageMsg();
            pub_debug_img_->publish(*debug_msg);
        }
    } catch (const std::exception & e) {
        (void)e;
    }
}

std::pair<double, double> BallDetectorNode::getBallPosition(
        double img_x, double img_y, int img_width, int img_height)
{
    double theta = -(img_x - img_width / 2.0) * hfov_rad / img_width + current_head_pan_;
    double phi = (img_y - img_height / 2.0) * vfov_rad / img_height + current_head_tilt_;
    double ux = std::cos(theta) * std::cos(phi);
    double uy = std::sin(theta) * std::cos(phi);
    double uz = -std::sin(phi);

    if (std::abs(uz) < 1e-6) {
        return {0.0, 0.0};
    }

    double lambda = (ball_radius_ - head_z_) / uz;
    double ball_x = head_x + lambda * ux;
    double ball_y = head_y + lambda * uy;
    return {ball_x, ball_y};
}
