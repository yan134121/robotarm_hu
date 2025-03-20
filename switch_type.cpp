#include "switch_type.h"


Pose DescPoseToPose(const DescPose& descPose) {
    Pose pose;

    // Position conversion (mm to m)
    pose.position.x = static_cast<float>(descPose.tran.x / 1000.0);
    pose.position.y = static_cast<float>(descPose.tran.y / 1000.0);
    pose.position.z = static_cast<float>(descPose.tran.z / 1000.0);

    // Euler angles conversion (deg to rad)
    pose.euler.rx = static_cast<float>(descPose.rpy.rx * DEG2RAD);
    pose.euler.ry = static_cast<float>(descPose.rpy.ry * DEG2RAD);
    pose.euler.rz = static_cast<float>(descPose.rpy.rz * DEG2RAD);

    // Convert Euler angles to Quaternion
    float crx = cos(pose.euler.rx / 2);
    float srx = sin(pose.euler.rx / 2);
    float cry = cos(pose.euler.ry / 2);
    float sry = sin(pose.euler.ry / 2);
    float crz = cos(pose.euler.rz / 2);
    float srz = sin(pose.euler.rz / 2);

    pose.quaternion.w = crx * cry * crz + srx * sry * srz;
    pose.quaternion.x = srx * cry * crz - crx * sry * srz;
    pose.quaternion.y = crx * sry * crz + srx * cry * srz;
    pose.quaternion.z = crx * cry * srz - srx * sry * crz;

    return pose;
}


DescPose PoseToDescPose(const Pose& pose) {
    DescPose descPose;

    // Position conversion (m to mm)
    descPose.tran.x = static_cast<double>(pose.position.x * 1000.0);
    descPose.tran.y = static_cast<double>(pose.position.y * 1000.0);
    descPose.tran.z = static_cast<double>(pose.position.z * 1000.0);

    // Convert Quaternion to Euler angles
    float qw = pose.quaternion.w;
    float qx = pose.quaternion.x;
    float qy = pose.quaternion.y;
    float qz = pose.quaternion.z;

    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    descPose.rpy.rx = atan2(sinr_cosp, cosr_cosp) * RAD2DEG;

    float sinp = 2 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        descPose.rpy.ry = copysign(M_PI / 2, sinp) * RAD2DEG; // Use 90 degrees if out of range
    else
        descPose.rpy.ry = asin(sinp) * RAD2DEG;

    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    descPose.rpy.rz = atan2(siny_cosp, cosy_cosp) * RAD2DEG;

    return descPose;
}
