#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QTimer>
#include <QDateTime>
#include <QMutex>
#include <QThread>
#include <QSettings>
#include <QDir>
#include <QPushButton>
#include <QMessageBox>
#include <QCheckBox>
#include <QVBoxLayout>

#include "rm_service.h"


#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <unistd.h>

#include "robot.h"
#include "robot_error.h"
#include "robot_types.h"

#include "Kinematics.h"
#include "switch_type.h"
#include <ostream>
#include "camera/cameraworker.h"
using namespace std;
using namespace cv;


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent *event) override; // 添加关闭事件处理


private slots:
    void statetimeout();
    void zeroTimeout();
    void FirstTimeout();
    void SenTimeout();
    void TriTimeout();
    void startmove();
    void FourTimeout();
    void on_modeswitch_clicked();
    void on_CloseGrip_clicked();
    void on_OpenGrip_clicked();
    DescPose Tool_IK_DRobot(DescPose pbt, DescTran pet);
    void on_ActArm_clicked();
    void on_CloseArm_clicked();
    void on_ActCamera_clicked();
    void on_MoveGrip_clicked();
    void on_ArgSetPush_clicked();
    void on_TeachCon_clicked();
    void on_ControlPush_clicked();
    void on_StorePos_clicked();
    void Modify_MechArm_clicked();

    //线程
    void updateCameraFrame(const QImage& image, int frameRate); // 更新 GUI
    void updatePoints(const QString& points);                   // 更新目标点坐标


    //获取配置参数
    void get_setting_value();
    //储存配置参数
    void store_setting_value();
    //初始化某些参数
    void init_parameters();
    //更新起点显示
    void updateoutset();
    //判断DescPose是否有效
    bool isDescPoseZero(const DescPose& pose, float epsilon = 1e-6f);
    //获取程序当前运行的目录
    QString getWeightPath();

    void on_joint1TeachDes_pressed();
    void on_joint1TeachInc_pressed();
    void on_joint2TeachDes_pressed();
    void on_joint2TeachInc_pressed();
    void on_joint3TeachDes_pressed();
    void on_joint3TeachInc_pressed();
    void on_joint4TeachDes_pressed();
    void on_joint4TeachInc_pressed();
    void on_joint5TeachDes_pressed();
    void on_joint5TeachInc_pressed();
    void on_joint6TeachDes_pressed();
    void on_joint6TeachInc_pressed();
    void on_joint1TeachDes_released();
    void on_joint1TeachInc_released();
    void on_joint2TeachDes_released();
    void on_joint2TeachInc_released();
    void on_joint3TeachDes_released();
    void on_joint3TeachInc_released();
    void on_joint4TeachDes_released();
    void on_joint4TeachInc_released();
    void on_joint5TeachDes_released();
    void on_joint5TeachInc_released();
    void on_joint6TeachDes_released();
    void on_joint6TeachInc_released();

    void on_pos_x_inc_pressed();
    void on_pos_x_des_pressed();
    void on_pos_y_inc_pressed();
    void on_pos_y_des_pressed();
    void on_pos_z_inc_pressed();
    void on_pos_z_des_pressed();
    void on_rollx_inc_pressed();
    void on_rollx_des_pressed();
    void on_pitchy_inc_pressed();
    void on_pitchy_des_pressed();
    void on_yawz_inc_pressed();
    void on_yawz_des_pressed();
    void on_pos_x_inc_released();
    void on_pos_x_des_released();
    void on_pos_y_inc_released();
    void on_pos_y_des_released();
    void on_pos_z_inc_released();
    void on_pos_z_des_released();
    void on_rollx_inc_released();
    void on_rollx_des_released();
    void on_pitchy_inc_released();
    void on_pitchy_des_released();
    void on_yawz_inc_released();
    void on_yawz_des_released();

    void on_PosDes_clicked();

    void on_closeCam_clicked();

    void on_JointDes_clicked();

    void on_MoveGripCon_clicked();

    void on_stop_clicked();

    void on_Set_storepoint_clicked();

    void on_Idle_Pos_clicked();

    void on_back_outset_clicked();




signals:
    void startCamera();  // 触发摄像头线程
    void stopCamera();           // 停止摄像头线程

private:
    Ui::MainWindow *ui;
    bool showPopupEnabled = true; // 默认显示弹窗
    FRRobot robot;//实 例 化 机 器 人 对 象
    QTimer *stateTime, *timer, *zeroTime, *FirstTime, *SenTime, *triTime, *fourTime, *fiveTime;
    QThread *cameraThread;       // 摄像头线程
    CameraWorker *cameraWorker;
    QStringList portList;
    bool congrip = false;
    std::string classNamesPath;
    std::vector<std::string> classNames;
    std::string modelPath;  // 模型路径
    //相机

    QImage color_image;//当前帧彩色图
    rs2_intrinsics intr;
    rs2_intrinsics depth_intrin;
    cv::Mat color_image_m;
    cv::Mat depth_image_m;
    int frameCnt = 0;//计算帧率
    QDateTime ct;

    //机械臂
    bool detectJ = true;
    QList<DescTran> points_obj;
    QList<float> points_z;
    QMutex mutex;//线程锁，避免points_obj被同时操作
    Eigen::Matrix4d T_c_g;
    bool get_obj_pose = false;
    uint8_t cameraflag = 0;
    Eigen::Matrix4d T_end_c ; //机械臂末端坐标系下相机的旋转矩阵
    //轨迹规划
    int speed;
    int speed_joint;
    float last_angle[6]={0,0,0,0,0,0};
    int flag = 0;//0-阻 塞 ， 1-非 阻 塞
    JointPos cur_angle_, j_rad;
    DescPose cur_pose_;//当前姿态
    DescPose outset;//抓取目标时的起点

    uint8_t Arm_Err;//机械臂错误代码
    uint16_t Sys_Err;


    bool targetJ = false;
    JointPos last_angle_;//储存相机识别到目标，准备抓取前的位置
    DescPose last_pose_temp;//储存相机识别到目标，准备抓取前的位置
    DescPose tar_robotend;//当前抓取目标位置
    DescPose storepoint;//存储点位姿
    bool up = false;

    DescTran Jaw;//夹爪相对机械臂末端的位移

//    //预备点补偿
//    float ResPointIncX;float ResPointIncY;float ResPointIncZ;
//    //目标点补偿
//    float TarPointIncX;float TarPointIncY;float TarPointIncZ;
//    //下压点补偿
//    float PrePointIncX;float PrePointIncY;float PrePointIncZ;
    //初始化位置
    float StartPosRX;float StartPosRY;float StartPosRZ;
    //使能标志
    uint8_t enableflag ;

    //机械臂运动参数
    int tool = 0;
    int user = 0;
    float acc = 100.0;
    float ovl = 50.0;
    float blendT = 200.0;
    float vel = 20  ;
    float blendR = 200.0;
    uint8_t search = 0;
    ExaxisPos epos;
};
#endif // MAINWINDOW_H
