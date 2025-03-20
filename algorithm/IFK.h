#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>
#include <eigen3/Eigen/Dense>
#define EPSINON 1e-4  // 精度，小于它就认为是0

static double PI = 3.141592653589793;   // 定义常数 pi
static double Angle_max[6] = { PI,PI, PI, PI, PI, PI, };
static double Angle_min[6] = { -PI,-PI,-PI,-PI,-PI,-PI };
static std::vector<double> OriginalState = { 0,PI / 2,0,-1 * PI / 2,0,0 };

//定义欧拉角
//欧拉角使用旋转顺序为x,y,z的欧拉角
class EulerAngle
{

public:
    double rollX, pitchY, yawZ;//欧拉角
    EulerAngle(double m_rollX = 0, double m_pitchY = 0, double m_yawZ = 0)
        :rollX(m_rollX), pitchY(m_pitchY), yawZ(m_yawZ) {};

};

//定义空间坐标
class coord
{
public:
    double x, y, z;
    coord(double m_x = 0, double m_y = 0, double m_z = 0)
        :x(m_x), y(m_y), z(m_z) {};
};


//定义一个包含姿态，位置的类
class posture
{
public:
    // posture类的构造函数
    EulerAngle angle;
    coord coordinate;

    posture(double rollX = 0, double pitchY = 0, double yawZ = 0, double x = 0, double y = 0, double z = 0)
        : angle(rollX, pitchY, yawZ), coordinate(x, y, z) {};
    posture(EulerAngle m_angle, coord m_coordinate)
        :angle(m_angle), coordinate(m_coordinate) {};
};






//函数
posture FK(const double* Theta);
posture FK_vertical(const double* Theta);
Eigen::Matrix4d FK_vertical_M(const double* Theta);
std::vector<std::vector<double>> IK(posture p);
std::vector<std::vector<double>> IK_vertical(posture p);
std::vector<double> BestSolution(std::vector<std::vector<double>>& solution, const std::vector<double>& PresentState);
coord Getcoord(Eigen::Matrix4d T);
EulerAngle RotationMatrixtoEulerAngle(Eigen::Matrix4d T);
Eigen::Matrix3d EulerAngletoRotationMatrix(EulerAngle Pose);
Eigen::Matrix4d postureToMatrix4d(posture p);
posture MatrixToposture(Eigen::Matrix4d T);
float rpm_conversion(float rpm,float time_step);
float rpm_count(float angle,float time_step);
float inc_euler(float incS,float eulerS);
float mid_euler(float eulerS);
