#include "new_ball_detector/ball_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>                      // cv::undistortPoints
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>          // CameraInfo subscription
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

// ============================================================================
//  REQUIRED ADDITIONS TO ball_detector.hpp
// ----------------------------------------------------------------------------
//  #include <opencv2/core.hpp>
//  #include <sensor_msgs/msg/camera_info.hpp>
//
//  // --- new member functions ---
//  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
//  cv::Point2f undistortPixel(float u, float v) const;
//  std::pair<double, double> getBallPosition(
//          double img_x, double img_y,
//          double bbox_w, double bbox_h,
//          int img_width, int img_height);   // <-- new signature (adds w, h)
//
//  // --- new member variables ---
//  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;
//  std::string camera_info_topic_;
//  bool   have_camera_info_{false};
//  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
//  cv::Mat K_, D_;
//
//  NOTE: the CameraInfo and image callbacks are assumed to run on the same
//        (default, single-threaded) executor, so no locking is used. If you
//        switch to a MultiThreadedExecutor, guard K_/D_/have_camera_info_
//        with a mutex.
// ============================================================================

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
    this->declare_parameter("camera_info_topic", "/boostercamera/head/camera_info");
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
    this->get_parameter("camera_info_topic", camera_info_topic_);
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
            std::bind(&BallDetectorNode::imageCallback, this, std::placeholders::_1));

    // Camera intrinsics + distortion (used to undistort detections)
    sub_cam_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, rclcpp::QoS(1),
            std::bind(&BallDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

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

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(
            this->get_logger(),
            "Ball topics: /vision/ball (%s), %s (map Pose2D)",
            base_frame_.c_str(), ball_map_topic_.c_str());
    RCLCPP_INFO(
            this->get_logger(),
            "Waiting for CameraInfo on '%s' (falls back to FOV intrinsics until received)",
            camera_info_topic_.c_str());
}

void BallDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    // k[0]=fx, k[2]=cx, k[4]=fy, k[5]=cy. A zero fx means "uncalibrated".
    if (msg->k[0] <= 0.0) {
        return;
    }

    fx_ = msg->k[0];
    cx_ = msg->k[2];
    fy_ = msg->k[4];
    cy_ = msg->k[5];

    K_ = (cv::Mat_<double>(3, 3) <<
            fx_, 0.0, cx_,
            0.0, fy_, cy_,
            0.0, 0.0, 1.0);

    if (!msg->d.empty()) {
        D_ = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
        for (size_t i = 0; i < msg->d.size(); ++i) {
            D_.at<double>(0, static_cast<int>(i)) = msg->d[i];
        }
    } else {
        D_ = cv::Mat::zeros(1, 5, CV_64F);
    }

    if (!have_camera_info_) {
        RCLCPP_INFO(
                this->get_logger(),
                "CameraInfo received: fx=%.1f fy=%.1f cx=%.1f cy=%.1f (%zu distortion coeffs)",
                fx_, fy_, cx_, cy_, msg->d.size());
    }
    have_camera_info_ = true;
}

// Undistort one pixel back to an ideal pinhole image (returns PIXEL coords).
// If no CameraInfo has arrived yet, the point is returned unchanged.
cv::Point2f BallDetectorNode::undistortPixel(float u, float v) const
{
    if (!have_camera_info_ || K_.empty()) {
        return {u, v};
    }
    std::vector<cv::Point2f> src{cv::Point2f(u, v)};
    std::vector<cv::Point2f> dst;
    // P = K_ so the output is expressed in pixels of the undistorted image.
    cv::undistortPoints(src, dst, K_, D_, cv::noArray(), K_);
    return dst.empty() ? cv::Point2f(u, v) : dst[0];
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

    // ======================================

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "pumas_base_link";
    t.child_frame_id = "pumas_ball";

    t.transform.translation.x = bx;
    t.transform.translation.y = by;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    if (tf_broadcaster_) {
        tf_broadcaster_->sendTransform(t);
    }
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

double BallDetectorNode::computeAngle(double img_x, int img_width)
{
    // Prefer the measured focal length when CameraInfo is available.
    const double f_x = have_camera_info_
            ? fx_
            : (img_width / 2.0) / std::tan(hfov_rad / 2.0);

    const double img_cx = have_camera_info_ ? cx_ : (img_width / 2.0);
    const double delta_x = img_cx - img_x;
    const double alpha_cam = std::atan2(delta_x, f_x);

    const double alpha_body = alpha_cam + current_head_pan_;

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
                auto [bx, by] = getBallPosition(
                        d.center_x, d.center_y, d.width, d.height, bgr.cols, bgr.rows);

                // getBallPosition returns {0,0} when it cannot form an estimate.
                if (bx == 0.0 && by == 0.0) {
                    continue;
                }

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

// ---------------------------------------------------------------------------
//  Ball position estimate.
//
//  Range is estimated primarily from the ball's APPARENT SIZE (independent of
//  head tilt, head_z and VFOV) and cross-checked against the ground-plane
//  intersection. The bearing/elevation use the true pinhole model, and every
//  pixel is undistorted first (needs CameraInfo; falls back gracefully).
//
//  Returns {ball_x, ball_y} in base_frame_ (x forward, y left), or {0,0} if
//  no usable estimate could be formed.
// ---------------------------------------------------------------------------
std::pair<double, double> BallDetectorNode::getBallPosition(
        double img_x, double img_y,
        double bbox_w, double bbox_h,
        int img_width, int img_height)
{
    // --- Effective intrinsics: prefer measured CameraInfo, else derive from FOV ---
    double fx, fy, cx, cy;
    if (have_camera_info_) {
        fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    } else {
        cx = img_width  / 2.0;
        cy = img_height / 2.0;
        fx = (img_width  / 2.0) / std::tan(hfov_rad / 2.0);
        fy = (img_height / 2.0) / std::tan(vfov_rad / 2.0);
    }

    // --- Undistort the detection: center + the four bbox edge-midpoints ---
    const cv::Point2f c_u = undistortPixel(img_x, img_y);
    const cv::Point2f l_u = undistortPixel(img_x - bbox_w / 2.0f, img_y);
    const cv::Point2f r_u = undistortPixel(img_x + bbox_w / 2.0f, img_y);
    const cv::Point2f t_u = undistortPixel(img_x, img_y - bbox_h / 2.0f);
    const cv::Point2f b_u = undistortPixel(img_x, img_y + bbox_h / 2.0f);

    const double u = c_u.x;
    const double v = c_u.y;
    const double und_w = std::hypot(r_u.x - l_u.x, r_u.y - l_u.y);  // corrected diameter (x)
    const double und_h = std::hypot(b_u.x - t_u.x, b_u.y - t_u.y);  // corrected diameter (y)

    // --- Bearing (theta, + to the left) and elevation (phi, + downward) ---
    const double theta = std::atan2(cx - u, fx) + current_head_pan_;
    const double phi   = std::atan2(v - cy, fy) + current_head_tilt_;

    const double cphi = std::cos(phi);
    const double ux = std::cos(theta) * cphi;   // forward (base +x)
    const double uy = std::sin(theta) * cphi;   // left    (base +y)
    const double uz = -std::sin(phi);           // up      (base +z); (ux,uy,uz) is a unit ray

    // --- Clipping check on the ORIGINAL bbox: a ball touching the frame edge
    //     has an unreliable apparent size, so the size cue must be dropped. ---
    const double m = 2.0;
    const double x1 = img_x - bbox_w / 2.0, y1 = img_y - bbox_h / 2.0;
    const double x2 = img_x + bbox_w / 2.0, y2 = img_y + bbox_h / 2.0;
    const bool clipped =
        (x1 <= m) || (y1 <= m) ||
        (x2 >= img_width  - m) || (y2 >= img_height - m);

    // --- Range #1: apparent size. Robust to tilt / head_z / VFOV errors. ---
    double R_size = -1.0;
    if (!clipped) {
        const double d_px = 0.5 * (und_w + und_h);          // avg apparent diameter
        if (d_px > 2.0) {
            const double alpha = std::atan2(d_px / 2.0, fx); // angular radius
            if (alpha > 1e-4) {
                R_size = ball_radius_ / std::sin(alpha);     // Euclidean dist to center
            }
        }
    }

    // --- Range #2: ground-plane intersection. Needs good tilt & head_z. ---
    double R_plane = -1.0;
    if (std::abs(uz) > 1e-6) {
        const double lambda = (ball_radius_ - head_z_) / uz;
        if (lambda > 0.0) {
            R_plane = lambda;   // lambda is Euclidean because (ux,uy,uz) is a unit vector
        }
    }

    // --- Fuse: average when they agree; otherwise trust the size estimate,
    //     which is immune to tilt / head_z drift. ---
    double R;
    if (R_size > 0.0 && R_plane > 0.0) {
        const double rel = std::abs(R_size - R_plane) / std::max(R_size, R_plane);
        R = (rel < 0.35) ? 0.5 * (R_size + R_plane) : R_size;
    } else if (R_size > 0.0) {
        R = R_size;
    } else if (R_plane > 0.0) {
        R = R_plane;
    } else {
        return {0.0, 0.0};
    }

    const double ball_x = head_x + R * ux;
    const double ball_y = head_y + R * uy;
    return {ball_x, ball_y};
}
