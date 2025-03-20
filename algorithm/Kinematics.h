#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <robot.h>
#include <robot_error.h>
#include <robot_types.h>
#define PI 3.1415926
Eigen::Matrix4d DescPosetoMatrix(const DescPose& pose);
DescPose matrixToDescPose(const Eigen::Matrix4d& mat);

#endif // KINEMATICS_H
