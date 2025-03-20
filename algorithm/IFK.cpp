#include "IFK.h"

Eigen::Matrix4d rotx(double angle);
Eigen::Matrix4d roty(double angle);
Eigen::Matrix4d rotz(double angle);
Eigen::Matrix4d trans(double x, double y, double z);
double deg2rad(double degree);




//函数功能：正运动学求解
//函数输入：一个6*1的关节角度数组,单位为弧度
//函数输出：一个4*4的齐次变换矩阵 
posture FK(const double* Theta)
{
    //判断输入是否为空指针
    assert(Theta != nullptr && "Input array cannot be nullptr.");

    // 检查输入数组的大小是否为6
    for (int i = 0; i < 6; ++i) {
        assert(!std::isnan(Theta[i]) && "Input array must not contain NaN.");
    }

    Eigen::Matrix4d T01 = rotz(Theta[0]) * trans(0, 0, d[0]) * rotx(alpha[0]) * trans(a[0], 0, 0);
    Eigen::Matrix4d T12 = rotz(Theta[1]) * trans(0, 0, d[1]) * rotx(alpha[1]) * trans(a[1], 0, 0);
    Eigen::Matrix4d T23 = rotz(Theta[2]) * trans(0, 0, d[2]) * rotx(alpha[2]) * trans(a[2], 0, 0);
    Eigen::Matrix4d T34 = rotz(Theta[3]) * trans(0, 0, d[3]) * rotx(alpha[3]) * trans(a[3], 0, 0);
    Eigen::Matrix4d T45 = rotz(Theta[4]) * trans(0, 0, d[4]) * rotx(alpha[4]) * trans(a[4], 0, 0);
    Eigen::Matrix4d T56 = rotz(Theta[5]) * trans(0, 0, d[5]) * rotx(alpha[5]) * trans(a[5], 0, 0);
    Eigen::Matrix4d T = T01 * T12 * T23 * T34 * T45 * T56;
    //计算cos和sin时有误差，将小于EPSINON的数设为0
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (abs(T(i, j)) < EPSINON)
            {
                T(i, j) = 0;
            }
        }
    }

    posture temp_posture = MatrixToposture(T);
    return temp_posture;

}

posture FK_vertical(const double* Theta)
{
    //判断输入是否为空指针
    assert(Theta != nullptr && "Input array cannot be nullptr.");

    // 检查输入数组的大小是否为6
    for (int i = 0; i < 6; ++i) {
        assert(!std::isnan(Theta[i]) && "Input array must not contain NaN.");
    }

    Eigen::Matrix4d T01 = rotz(Theta[0]) * trans(0, 0, d[0]) * rotx(alpha[0]) * trans(a[0], 0, 0);
    Eigen::Matrix4d T12 = rotz(Theta[1]-1.5707) * trans(0, 0, d[1]) * rotx(alpha[1]) * trans(a[1], 0, 0);
    Eigen::Matrix4d T23 = rotz(Theta[2]) * trans(0, 0, d[2]) * rotx(alpha[2]) * trans(a[2], 0, 0);
    Eigen::Matrix4d T34 = rotz(Theta[3]-1.5707) * trans(0, 0, d[3]) * rotx(alpha[3]) * trans(a[3], 0, 0);
    Eigen::Matrix4d T45 = rotz(Theta[4]) * trans(0, 0, d[4]) * rotx(alpha[4]) * trans(a[4], 0, 0);
    Eigen::Matrix4d T56 = rotz(Theta[5]) * trans(0, 0, d[5]) * rotx(alpha[5]) * trans(a[5], 0, 0);
    Eigen::Matrix4d T = T01 * T12 * T23 * T34 * T45 * T56;
    //计算cos和sin时有误差，将小于EPSINON的数设为0
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (abs(T(i, j)) < EPSINON)
            {
                T(i, j) = 0;
            }
        }
    }

    posture temp_posture = MatrixToposture(T);
    return temp_posture;

}

Eigen::Matrix4d FK_vertical_M(const double* Theta)
{
    //判断输入是否为空指针
    assert(Theta != nullptr && "Input array cannot be nullptr.");

    // 检查输入数组的大小是否为6
    for (int i = 0; i < 6; ++i) {
        assert(!std::isnan(Theta[i]) && "Input array must not contain NaN.");
    }

    Eigen::Matrix4d T01 = rotz(Theta[0]) * trans(0, 0, d[0]) * rotx(alpha[0]) * trans(a[0], 0, 0);
    Eigen::Matrix4d T12 = rotz(Theta[1]-1.5707) * trans(0, 0, d[1]) * rotx(alpha[1]) * trans(a[1], 0, 0);
    Eigen::Matrix4d T23 = rotz(Theta[2]) * trans(0, 0, d[2]) * rotx(alpha[2]) * trans(a[2], 0, 0);
    Eigen::Matrix4d T34 = rotz(Theta[3]-1.5707) * trans(0, 0, d[3]) * rotx(alpha[3]) * trans(a[3], 0, 0);
    Eigen::Matrix4d T45 = rotz(Theta[4]) * trans(0, 0, d[4]) * rotx(alpha[4]) * trans(a[4], 0, 0);
    Eigen::Matrix4d T56 = rotz(Theta[5]) * trans(0, 0, d[5]) * rotx(alpha[5]) * trans(a[5], 0, 0);
    Eigen::Matrix4d T = T01 * T12 * T23 * T34 * T45 * T56;
    //计算cos和sin时有误差，将小于EPSINON的数设为0
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            if (abs(T(i, j)) < EPSINON)
            {
                T(i, j) = 0;
            }
        }
    }
    return T;

}

//函数功能：逆运动学求解
//函数输入：基坐标系到机械臂末端坐标系的姿态变换
//函数输出：8*6的vector数组，每一行表示一种解
std::vector<std::vector<double>> IK(posture p)
{
    //把位置和欧拉角转换为变换矩阵

    Eigen::Matrix4d T = postureToMatrix4d(p);

    // 定义一个8x6的二维 std::vector
    std::vector<std::vector<double>> jointAngles(8, std::vector<double>(6, 0.0));

    //将其次变换矩阵各参数简写，方便后面阅读
    double  nx = T(0, 0), ox = T(0, 1), ax = T(0, 2),
        ny = T(1, 0), oy = T(1, 1), ay = T(1, 2),
        nz = T(2, 0), oz = T(2, 1), az = T(2, 2),
        px = T(0, 3), py = T(1, 3), pz = T(2, 3);

    // 求解theta1
    double m = d[5] * ay - py;
    double n = ax * d[5] - px;
    std::vector<double> theta1(8, 0.0);

    for (int j = 0; j < 4; j++)
    {
        theta1[j] = atan2(m, n) - atan2(d[3], sqrt(m * m + n * n - d[3] * d[3]));
        theta1[j + 4] = atan2(m, n) - atan2(d[3], -sqrt(m * m + n * n - d[3] * d[3]));
    }

    // 求解theta5
    std::vector<double> theta5(8, 0.0);
    theta5[0] = acos(ax * sin(theta1[0]) - ay * cos(theta1[0]));
    theta5[1] = theta5[0];
    theta5[2] = -1 * theta5[0];
    theta5[3] = theta5[2];
    theta5[4] = acos(ax * sin(theta1[4]) - ay * cos(theta1[4]));
    theta5[5] = theta5[4];
    theta5[6] = -1 * theta5[4];
    theta5[7] = theta5[6];

    // 求解theta6
    std::vector<double> theta6(8, 0.0);
    for (int j = 0; j < 8; j++)
    {
        double m = nx * sin(theta1[j]) - ny * cos(theta1[j]);
        double n = ox * sin(theta1[j]) - oy * cos(theta1[j]);
        theta6[j] = atan2(m, n) - atan2(sin(theta5[j]), 0);
    }

    // 求解theta3
    std::vector<double> theta3(8, 0.0);
    std::vector<double> temp_m(8, 0.0);
    std::vector<double> temp_n(8, 0.0);
    for (int j = 0; j < 8; j++)
    {
        temp_m[j] = d[4] * (sin(theta6[j]) * (nx * cos(theta1[j]) + ny * sin(theta1[j])) + cos(theta6[j]) * (ox * cos(theta1[j]) + oy * sin(theta1[j]))) - d[5] * (ax * cos(theta1[j]) + ay * sin(theta1[j])) + px * cos(theta1[j]) + py * sin(theta1[j]);
        temp_n[j] = pz - d[0] - az * d[5] + d[4] * (oz * cos(theta6[j]) + nz * sin(theta6[j]));
        if (j % 2 == 0) {
            theta3[j] = acos((temp_m[j] * temp_m[j] + temp_n[j] * temp_n[j] - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]));
        }
        else {
            theta3[j] = -1 * acos((temp_m[j] * temp_m[j] + temp_n[j] * temp_n[j] - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]));
        }
    }

    // 求解theta2
    std::vector<double> theta2(8, 0.0);
    std::vector<double> s2(8, 0.0);
    std::vector<double> c2(8, 0.0);
    for (int j = 0; j < 8; j++)
    {
        s2[j] = ((a[2] * cos(theta3[j]) + a[1]) * temp_n[j] - a[2] * sin(theta3[j]) * temp_m[j]) / (a[1] * a[1] + a[2] * a[2] + 2 * a[1] * a[2] * cos(theta3[j]));
        c2[j] = (temp_m[j] + a[2] * sin(theta3[j]) * s2[j]) / (a[2] * cos(theta3[j]) + a[1]);
        theta2[j] = atan2(s2[j], c2[j]);
    }

    // 求解theta4
    std::vector<double> theta4(8, 0.0);
    for (int j = 0; j < 8; j++)
    {
        theta4[j] = atan2(-1 * sin(theta6[j]) * (nx * cos(theta1[j]) + ny * sin(theta1[j])) - cos(theta6[j]) * (ox * cos(theta1[j]) + oy * sin(theta1[j])), oz * cos(theta6[j]) + nz * sin(theta6[j])) - theta2[j] - theta3[j];
    }

    // 存储逆解结果
    for (int j = 0; j < 8; j++)
    {
        jointAngles[j][0] = theta1[j];
        jointAngles[j][1] = theta2[j];
        jointAngles[j][2] = theta3[j];
        jointAngles[j][3] = theta4[j];
        jointAngles[j][4] = theta5[j];
        jointAngles[j][5] = theta6[j];
    }
    // 限制关节角度在 -π 到 π 之间
    //for (int i = 0; i < 8; i++) {
    //    for (int j = 0; j < 6; j++) {
    //        while (jointAngles[i][j] < -PI) {
    //            jointAngles[i][j] += 2 * PI;
    //        }
    //        while (jointAngles[i][j] > PI) {
    //            jointAngles[i][j] -= 2 * PI;
    //        }
    //    }
    //}
    return jointAngles;
}

std::vector<std::vector<double>> IK_vertical(posture p)
{
    //把位置和欧拉角转换为变换矩阵

    Eigen::Matrix4d T = postureToMatrix4d(p);

    // 定义一个8x6的二维 std::vector
    std::vector<std::vector<double>> jointAngles(8, std::vector<double>(6, 0.0));

    //将其次变换矩阵各参数简写，方便后面阅读
    double  nx = T(0, 0), ox = T(0, 1), ax = T(0, 2),
        ny = T(1, 0), oy = T(1, 1), ay = T(1, 2),
        nz = T(2, 0), oz = T(2, 1), az = T(2, 2),
        px = T(0, 3), py = T(1, 3), pz = T(2, 3);

    // 求解theta1
    double m = d[5] * ay - py;
    double n = ax * d[5] - px;
    std::vector<double> theta1(8, 0.0);

    for (int j = 0; j < 4; j++)
    {
        theta1[j] = atan2(m, n) - atan2(d[3], sqrt(m * m + n * n - d[3] * d[3]));
        theta1[j + 4] = atan2(m, n) - atan2(d[3], -sqrt(m * m + n * n - d[3] * d[3]));
    }

    // 求解theta5
    std::vector<double> theta5(8, 0.0);
    theta5[0] = acos(ax * sin(theta1[0]) - ay * cos(theta1[0]));
    theta5[1] = theta5[0];
    theta5[2] = -1 * theta5[0];
    theta5[3] = theta5[2];
    theta5[4] = acos(ax * sin(theta1[4]) - ay * cos(theta1[4]));
    theta5[5] = theta5[4];
    theta5[6] = -1 * theta5[4];
    theta5[7] = theta5[6];

    // 求解theta6
    std::vector<double> theta6(8, 0.0);
    for (int j = 0; j < 8; j++)
    {
        double m = nx * sin(theta1[j]) - ny * cos(theta1[j]);
        double n = ox * sin(theta1[j]) - oy * cos(theta1[j]);
        theta6[j] = atan2(m, n) - atan2(sin(theta5[j]), 0);
    }

    // 求解theta3
    std::vector<double> theta3(8, 0.0);
    std::vector<double> temp_m(8, 0.0);
    std::vector<double> temp_n(8, 0.0);
    for (int j = 0; j < 8; j++)
    {
        temp_m[j] = d[4] * (sin(theta6[j]) * (nx * cos(theta1[j]) + ny * sin(theta1[j])) + cos(theta6[j]) * (ox * cos(theta1[j]) + oy * sin(theta1[j]))) - d[5] * (ax * cos(theta1[j]) + ay * sin(theta1[j])) + px * cos(theta1[j]) + py * sin(theta1[j]);
        temp_n[j] = pz - d[0] - az * d[5] + d[4] * (oz * cos(theta6[j]) + nz * sin(theta6[j]));
        if (j % 2 == 0) {
            theta3[j] = acos((temp_m[j] * temp_m[j] + temp_n[j] * temp_n[j] - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]));
        }
        else {
            theta3[j] = -1 * acos((temp_m[j] * temp_m[j] + temp_n[j] * temp_n[j] - a[1] * a[1] - a[2] * a[2]) / (2 * a[1] * a[2]));
        }
    }

    // 求解theta2
    std::vector<double> theta2(8, 0.0);
    std::vector<double> s2(8, 0.0);
    std::vector<double> c2(8, 0.0);
    for (int j = 0; j < 8; j++)
    {
        s2[j] = ((a[2] * cos(theta3[j]) + a[1]) * temp_n[j] - a[2] * sin(theta3[j]) * temp_m[j]) / (a[1] * a[1] + a[2] * a[2] + 2 * a[1] * a[2] * cos(theta3[j]));
        c2[j] = (temp_m[j] + a[2] * sin(theta3[j]) * s2[j]) / (a[2] * cos(theta3[j]) + a[1]);
        theta2[j] = atan2(s2[j], c2[j]);
    }

    // 求解theta4
    std::vector<double> theta4(8, 0.0);
    for (int j = 0; j < 8; j++)
    {
        theta4[j] = atan2(-1 * sin(theta6[j]) * (nx * cos(theta1[j]) + ny * sin(theta1[j])) - cos(theta6[j]) * (ox * cos(theta1[j]) + oy * sin(theta1[j])), oz * cos(theta6[j]) + nz * sin(theta6[j])) - theta2[j] - theta3[j];
    }

    // 存储逆解结果
    for (int j = 0; j < 8; j++)
    {
        jointAngles[j][0] = theta1[j];
        jointAngles[j][1] = theta2[j]+1.5707;
        jointAngles[j][2] = theta3[j];
        jointAngles[j][3] = theta4[j]+1.5707;
        jointAngles[j][4] = theta5[j];
        jointAngles[j][5] = theta6[j];
    }
    // 限制关节角度在 -π 到 π 之间
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 6; j++) {
            while (jointAngles[i][j] < -PI) {
                jointAngles[i][j] += 2 * PI;
            }
            while (jointAngles[i][j] > PI) {
                jointAngles[i][j] -= 2 * PI;
            }
        }
    }
    return jointAngles;
}


//函数功能：分别从各关节角是否符合用户所定义的最大/小关节要求，然后判断各关节角与当前关节角的差（转角大小）作为评价标准,最后决定最优解
//函数输入：IK函数的返回数组（表示逆运行学的所有解），机械臂此时的各关节角度数组,定义初始各关节角数组为OriginalState
//函数输出：1*6的数组，表示各个关节角的旋转角度或者Nan（全部不符合）
std::vector<double> BestSolution(std::vector<std::vector<double>>& solution, const std::vector<double>& PresentState)
{
    //首先判断每一组解的每一个关节都在取值范围内
    bool IsAngleOk[8] = { true,true,true,true,true,true,true,true };
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (solution[i][j]<Angle_max[j] && solution[i][j] > Angle_min[j]);
            else
                IsAngleOk[i] = false;

        }
    }
    double score[8] = { 0,0,0,0,0,0,0,0 };//
    double weight[6] = { 1,1,1,1,1,1 };//各个关节的权重

    for (int i = 0; i < 8; i++)
    {
        if (IsAngleOk[i] = true)
        {
            for (int j = 0; j < 6; j++)
            {
                score[i] += weight[j] * abs(solution[i][j] - PresentState[j]);
            }
        }
        else
            score[i] = -1;
    }

    double score_min = 100000;
    int flag = -1;
    for (int i = 0; i < 8; i++)
    {
        if (score[i] != -1 && score[i] < score_min)
        {

            score_min = score[i];
            flag = i;
        }

    }
    if (flag != -1)
    {
        return solution[flag];
    }
    else
    {
        return std::vector<double>(PresentState.size(), std::numeric_limits<double>::quiet_NaN());
    }

}


//函数功能：将旋转矩阵转换为欧拉角
//函数输入：4×4的位姿矩阵
//函数输出：Pose的class，包含rollX, pitchY，yawZ 的欧拉角（弧度角）
EulerAngle RotationMatrixtoEulerAngle(Eigen::Matrix4d T)
{
    EulerAngle Pose;
    Pose.pitchY = asin(T(0, 2));
    //如果绕Y的旋转角度为±90°，则需要特殊分析
    if (abs(Pose.pitchY - PI / 2) < 1e-6)
    {
        Pose.rollX = 0;
        Pose.pitchY = atan2(T(1, 0), T(1, 1));
    }
    else if (abs(Pose.pitchY + PI / 2) < 1e-6)
    {
        Pose.rollX = 0;
        Pose.pitchY = atan2(-T(1, 0), -T(1, 1));
    }
    else
    {
        Pose.yawZ = atan2(-T(0, 1), T(0, 0));
        Pose.rollX = atan2(-T(1, 2), T(2, 2));
    }
    return Pose;

}
//EulerAngle RotationMatrixtoEulerAngle(Eigen::Matrix4d T)
//{
//    EulerAngle Pose;
//    float sy;
//    sy = sqrt(T(0,0)*T(0,0)+T(1,0)*T(1,0));
//    //如果绕Y的旋转角度为±90°，则需要特殊分析
//    if (sy > 1e-6)
//    {
//        Pose.rollX = atan2(T(2,1),T(2,2));
//        Pose.pitchY = atan2(-1*T(2, 0), sy);
//        Pose.yawZ = atan2(T(1,0),T(0,0));
//    }
//    else
//    {
//        Pose.yawZ = 0;
//        Pose.rollX = atan2(-T(1, 2), T(1, 1));
//        Pose.pitchY = atan2(-T(2, 0), sy);
//    }
//    return Pose;

//}


//函数功能：将欧拉角转换为旋转矩阵
//函数输入：Pose的class，包含rollX, pitchY，yawZ 的欧拉角（弧度角）
//函数输出：3×3的旋转矩阵
Eigen::Matrix3d EulerAngletoRotationMatrix(EulerAngle Pose)
{
    double s1 = sin(Pose.rollX);
    double s2 = sin(Pose.pitchY);
    double s3 = sin(Pose.yawZ);
    double c1 = cos(Pose.rollX);
    double c2 = cos(Pose.pitchY);
    double c3 = cos(Pose.yawZ);

    Eigen::Matrix3d T;

    T(0, 0) = c2 * c3;
    T(0, 1) = -1 * c2 * s3;
    T(0, 2) = s2;
    T(1, 0) = s1 * s2 * c3 + c1 * s3;
    T(1, 1) = -1 * s1 * s2 * s3 + c1 * c3;
    T(1, 2) = -1 * s1 * c2;
    T(2, 0) = -c1 * s2 * c3 + s1 * s3;
    T(2, 1) = c1 * s2 * s3 + s1 * c3;
    T(2, 2) = c1 * c2;
    return T;
}

//Eigen::Matrix3d EulerAngletoRotationMatrix(EulerAngle Pose)
//{
//    double s1 = sin(Pose.rollX);
//    double s2 = sin(Pose.pitchY);
//    double s3 = sin(Pose.yawZ);
//    double c1 = cos(Pose.rollX);
//    double c2 = cos(Pose.pitchY);
//    double c3 = cos(Pose.yawZ);

//    Eigen::Matrix3d T;

//    T(0, 0) = c2 * c3;
//    T(1, 0) = c2 * s3;
//    T(2, 0) = -1 * s2;
//    T(0, 1) = s1 * s2 * c3 - c1 * s3;
//    T(1, 1) = s1 * s2 * s3 + c1 * c3;
//    T(2, 1) = s1 * c2;
//    T(0, 2) = c1 * s2 * c3 + s1 * s3;
//    T(1, 2) = c1 * s2 * s3 - s1 * c3;
//    T(2, 2) = c1 * c2;
//    return T;
//}

//函数功能：得到姿态的平移坐标
//函数输入：Matrix的位姿齐次矩阵
//函数输出：coord的class，包含x,y,z
coord Getcoord(Eigen::Matrix4d T)
{
    coord coord;
    coord.x = T(0, 3);
    coord.y = T(1, 3);
    coord.z = T(2, 3);
    return coord;

}

//函数功能：旋转矩阵函数
//函数输入：绕x轴旋转角度
//函数输出：4*4的绕x轴的旋转矩阵
Eigen::Matrix4d rotx(double angle)
{
    double c = cos(angle);
    double s = sin(angle);
    Eigen::Matrix4d T;
    T.setZero();
    T(0, 0) = 1;
    T(1, 1) = c;
    T(1, 2) = -s;
    T(2, 1) = s;
    T(2, 2) = c;
    T(3, 3) = 1;
    return T;
}



//函数功能：旋转矩阵函数
//函数输入：绕x轴旋转角度
//函数输出：4*4的绕x轴的旋转矩阵
Eigen::Matrix4d roty(double angle)
{
    double c = cos(angle);
    double s = sin(angle);
    Eigen::Matrix4d T;
    T.setZero();
    T(0, 0) = c;
    T(0, 2) = s;
    T(1, 1) = 1;
    T(2, 0) = -s;
    T(2, 2) = c;
    T(3, 3) = 1;
    return T;
}



//函数功能：旋转矩阵函数
//函数输入：绕x轴旋转角度
//函数输出：4*4的绕x轴的旋转矩阵
Eigen::Matrix4d rotz(double angle)
{
    double c = cos(angle);
    double s = sin(angle);
    Eigen::Matrix4d T;
    T.setZero();
    T(0, 0) = c;
    T(0, 1) = -s;
    T(1, 0) = s;
    T(1, 1) = c;
    T(2, 2) = 1;
    T(3, 3) = 1;
    return T;
}




//函数功能：平移矩阵
//函数输入：平移目标坐标x，y，z
//函数输出：4*4的平移矩阵
Eigen::Matrix4d trans(double x, double y, double z)
{
    Eigen::Matrix4d T;
    T.setZero();
    T(0, 0) = 1;
    T(0, 3) = x;
    T(1, 1) = 1;
    T(1, 3) = y;
    T(2, 2) = 1;
    T(2, 3) = z;
    T(3, 3) = 1;
    return T;
}




//函数功能：角度转弧度
//函数输入：角度
//函数输出：弧度
double deg2rad(double degree)
{
    return degree * PI / 2;
}


/*
函数功能：将位姿转换为4*4的旋转矩阵
函数输入：包含位姿和姿态的posture类
函数输出：Matrix4d 旋转矩阵
*/

Eigen::Matrix4d postureToMatrix4d(posture p)
{
    Eigen::Matrix3d T_posture = EulerAngletoRotationMatrix(p.angle);
    Eigen::Matrix4d T;
    T.block<3, 3>(0, 0) = T_posture;
    T(0, 3) = p.coordinate.x;
    T(1, 3) = p.coordinate.y;
    T(2, 3) = p.coordinate.z;
    T(3, 3) = 1;
    return T;
}


/*
函数功能：将4*4的旋转矩阵转换为位姿
函数输入：4*4的Matrix4d齐次变换矩阵
函数输出：位姿posture
*/
posture MatrixToposture(Eigen::Matrix4d T)
{
    posture p;

    p.angle = RotationMatrixtoEulerAngle(T);
    p.coordinate.x = T(0, 3);
    p.coordinate.y = T(1, 3);
    p.coordinate.z = T(2, 3);
    return p;

}

float rpm_conversion(float rpm,float time_step){
    float v_rpm;
    v_rpm = 360 * rpm / 60 /(1000/time_step);
    return v_rpm;
}

float rpm_count(float angle,float time_step){
    float rpm;
    rpm = angle * (1000/time_step) *60 /360;
    return rpm;
}

float inc_euler(float incS,float eulerS){
    if(abs(incS)>PI){
        if(eulerS<0){
            incS = incS - 2*PI;
        }
        else{
            incS = incS + 2*PI;
        }
        return incS;
    }
    else{
        return incS;
    }
}

float mid_euler(float eulerS){
    if(abs(eulerS)>PI){
        if(eulerS>0){
            eulerS = eulerS - 2*PI;
        }
        else{
            eulerS = eulerS + 2*PI;
        }
        return eulerS;
    }
    else{
        return eulerS;
    }
}
