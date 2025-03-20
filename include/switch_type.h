#ifndef SWITCH_TYPE_H
#define SWITCH_TYPE_H
#include <cmath>
#include <iostream>
#include "robot.h"
#include "robot_error.h"
#include "robot_types.h"
#include "rm_service.h"



// DescPose -> Pose
Pose DescPoseToPose(const DescPose& descPose);
// Pose -> DescPose
DescPose PoseToDescPose(const Pose& pose) ;


#endif // SWITCH_TYPE_H
