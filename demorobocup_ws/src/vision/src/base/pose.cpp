#include "booster_vision/base/pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace booster_vision {

Pose::Pose(const float &x, const float &y, const float &z,
           const float &roll, const float &pitch, const float &yaw) {
    cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                  0, cos(roll), -sin(roll),
                  0, sin(roll), cos(roll));

    cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch),
                  0, 1, 0,
                  -sin(pitch), 0, cos(pitch));

    cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0,
                  sin(yaw), cos(yaw), 0,
                  0, 0, 1);
    cv::Mat R = Rz * Ry * Rx;

    R.copyTo(mat_pose(cv::Rect(0, 0, 3, 3)));

    mat_pose.at<float>(0, 3) = x;
    mat_pose.at<float>(1, 3) = y;
    mat_pose.at<float>(2, 3) = z;
}

Pose::Pose(const cv::Mat& rot, const cv::Mat& trans) {
    if (rot.rows == 3 && rot.cols == 3) {
        rot.copyTo(mat_pose(cv::Rect(0, 0, 3, 3)));
    } else if (rot.rows == 3 && rot.cols == 1) {
        cv::Mat R;
        cv::Rodrigues(rot, R);
        R.copyTo(mat_pose(cv::Rect(0, 0, 3, 3)));
    } else {
        std::cerr << "Invalid rotation matrix size." << std::endl;
    }
    trans.copyTo(mat_pose(cv::Rect(3, 0, 1, 3)));
}

Pose::Pose(const geometry_msgs::msg::TransformStamped &msg) {
    mat_pose.at<float>(0, 3) = msg.transform.translation.x;
    mat_pose.at<float>(1, 3) = msg.transform.translation.y;
    mat_pose.at<float>(2, 3) = msg.transform.translation.z;

    tf2::Quaternion quaternion(msg.transform.rotation.x, msg.transform.rotation.y,
                               msg.transform.rotation.z, msg.transform.rotation.w);
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setRotation(quaternion);

    cv::Mat R = (cv::Mat_<float>(3, 3) << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
                 rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],
                 rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]);

    R.copyTo(mat_pose(cv::Rect(0, 0, 3, 3)));
}

Pose::Pose(float x, float y, float z,
           float qx, float qy, float qz, float qw) {
    mat_pose.at<float>(0, 3) = x;
    mat_pose.at<float>(1, 3) = y;
    mat_pose.at<float>(2, 3) = z;

    tf2::Quaternion quaternion(qx, qy, qz, qw);
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setRotation(quaternion);

    cv::Mat R = (cv::Mat_<float>(3, 3) << rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
                 rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],
                 rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]);

    R.copyTo(mat_pose(cv::Rect(0, 0, 3, 3)));
}

geometry_msgs::msg::TransformStamped Pose::toRosTFMsg() {
    geometry_msgs::msg::TransformStamped msg;

    msg.transform.translation.x = mat_pose.at<float>(0, 3);
    msg.transform.translation.y = mat_pose.at<float>(1, 3);
    msg.transform.translation.z = mat_pose.at<float>(2, 3);

    tf2::Matrix3x3 tf_rotation(mat_pose.at<float>(0, 0), mat_pose.at<float>(0, 1), mat_pose.at<float>(0, 2),
                               mat_pose.at<float>(1, 0), mat_pose.at<float>(1, 1), mat_pose.at<float>(1, 2),
                               mat_pose.at<float>(2, 0), mat_pose.at<float>(2, 1), mat_pose.at<float>(2, 2));
    tf2::Quaternion quaternion;
    tf_rotation.getRotation(quaternion);

    msg.transform.rotation.x = quaternion.x();
    msg.transform.rotation.y = quaternion.y();
    msg.transform.rotation.z = quaternion.z();
    msg.transform.rotation.w = quaternion.w();

    return msg;
}

std::vector<float> Pose::getEulerAnglesVec() const {
    cv::Mat R = getRotationMatrix();

    float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));
    bool singular = sy < 1e-6; // If true, we're at a singularity

    float x, y, z; // Roll, Pitch, Yaw
    if (!singular) {
        x = atan2(R.at<float>(2, 1), R.at<float>(2, 2)); // Yaw
        y = atan2(-R.at<float>(2, 0), sy);               // Pitch
        z = atan2(R.at<float>(1, 0), R.at<float>(0, 0)); // Roll
    } else {
        x = atan2(-R.at<float>(1, 2), R.at<float>(1, 1)); // Yaw
        y = atan2(-R.at<float>(2, 0), sy);                // Pitch
        z = 0;                                            // Roll is set to 0 in singularity case
    }

    return {x, y, z};
}

std::vector<double> Pose::getQuaternionVec() const {
    tf2::Matrix3x3 tf_rotation(mat_pose.at<float>(0, 0), mat_pose.at<float>(0, 1), mat_pose.at<float>(0, 2),
                               mat_pose.at<float>(1, 0), mat_pose.at<float>(1, 1), mat_pose.at<float>(1, 2),
                               mat_pose.at<float>(2, 0), mat_pose.at<float>(2, 1), mat_pose.at<float>(2, 2));
    tf2::Quaternion quaternion;
    tf_rotation.getRotation(quaternion);

    return {quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()};
}
} // namespace booster_vision