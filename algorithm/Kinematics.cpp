#include <Kinematics.h>

Eigen::Matrix4d DescPosetoMatrix(const DescPose& pose)
{
    // 提取位置
    Eigen::Vector3d translation(pose.tran.x, pose.tran.y, pose.tran.z);

    // 提取姿态
    double roll = pose.rpy.rx * M_PI/180;    // 滚转角
    double pitch = pose.rpy.ry * M_PI/180;   // 俯仰角
    double yaw = pose.rpy.rz * M_PI/180;    // 偏航角

    // 计算旋转矩阵（使用Tait-Bryan角，顺序：roll-pitch-yaw）
    Eigen::Matrix3d rotation;

    // 绕X轴旋转 (roll)
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
           0, cos(roll), -sin(roll),
           0, sin(roll), cos(roll);

    // 绕Y轴旋转 (pitch)
    Eigen::Matrix3d R_y;
    R_y << cos(pitch), 0, sin(pitch),
           0, 1, 0,
           -sin(pitch), 0, cos(pitch);

    // 绕Z轴旋转 (yaw)
    Eigen::Matrix3d R_z;
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw), cos(yaw), 0,
           0, 0, 1;

    // 组合旋转矩阵
    rotation = R_z * R_y * R_x;

    // 创建4x4齐次变换矩阵
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotation;
    transformationMatrix.block<3, 1>(0, 3) = translation;

    return transformationMatrix;
}

DescPose matrixToDescPose(const Eigen::Matrix4d& mat) {
    DescPose pose;

    // 提取平移部分
    pose.tran.x = mat(0, 3) ; // 单位转换为mm
    pose.tran.y = mat(1, 3) ;
    pose.tran.z = mat(2, 3) ;

    // 提取旋转矩阵
    Eigen::Matrix3d R = mat.block<3, 3>(0, 0);

    // 计算RPY角度 (ZYX顺序)
    pose.rpy.ry = std::asin(-R(2, 0)) * 180.0 / M_PI; // pitch
    if (std::abs(R(2, 0)) < 0.99999) {  // 避免万向锁
        pose.rpy.rx = std::atan2(R(2, 1), R(2, 2)) * 180.0 / M_PI; // roll
        pose.rpy.rz = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI; // yaw
    } else { // 发生万向锁，默认yaw = 0
        pose.rpy.rx = std::atan2(-R(1, 2), R(1, 1)) * 180.0 / M_PI;
        pose.rpy.rz = 0;
    }

    return pose;
}
