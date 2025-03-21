#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <chrono>
#include <iostream>

#include <librealsense2/rs.hpp>
#include <ostream>




const float confThreshold = 0.6f;  // 检测的置信度阈值
const float iouThreshold = 0.4f;   // NMS（非极大值抑制）的IOU阈值


YOLODetector detector{nullptr};  // 初始化YOLO检测器



// 休眠(毫秒)
void sleep_cp(int milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds);
#endif

#ifdef __linux
    usleep(milliseconds * 1000);
#endif
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //获取上一次保存的数据
    get_setting_value();
    //初始化一些参数
    init_parameters();
    //gui界面初始化
    ui->ArgInput->show();
    ui->ControlShow->show();
    ui->JointCon->hide();
    // 可选：设置初始状态
    // 设置gui界面中是否选择前往储存点和是否返回起点
    ui->isstore->setChecked(false); // 默认未选中
    ui->isoutset->setChecked(false); // 默认未选中


    timer = new QTimer(this);
    stateTime = new QTimer(this);

    connect(stateTime,SIGNAL(timeout()),SLOT(statetimeout()));
    Modify_MechArm_clicked();//从ui界面上获取当前机械臂的参数

    //机械臂连接信号槽
    zeroTime = new QTimer(this);
    connect(zeroTime,SIGNAL(timeout()),SLOT(zeroTimeout()));
    FirstTime = new QTimer(this);
    connect(FirstTime,SIGNAL(timeout()),SLOT(FirstTimeout()));
    SenTime = new QTimer(this);
    connect(SenTime,SIGNAL(timeout()),SLOT(SenTimeout()));
    triTime = new QTimer(this);
    connect(triTime,SIGNAL(timeout()),SLOT(TriTimeout()));
    fourTime = new QTimer(this);
    connect(fourTime,SIGNAL(timeout()),SLOT(FourTimeout()));
    fiveTime = new QTimer(this);
    connect(fiveTime,SIGNAL(timeout()),SLOT(startmove()));

    // 初始化摄像头线程
    cameraThread = new QThread(this);
    cameraWorker = new CameraWorker(modelPath, classNames, confThreshold,iouThreshold, points_obj, mutex, detectJ);
    cameraWorker->moveToThread(cameraThread);

    // 连接信号槽
    connect(this, &MainWindow::startCamera, cameraWorker, &CameraWorker::startCamera);
    connect(this, &MainWindow::stopCamera, cameraWorker, &CameraWorker::stopCamera);
    connect(cameraWorker, &CameraWorker::frameReady, this, &MainWindow::updateCameraFrame);
    connect(cameraWorker, &CameraWorker::pointsUpdated, this, &MainWindow::updatePoints);
    connect(cameraWorker, &CameraWorker::centerPointUpdated, this, &MainWindow::on_centerPointUpdated);
    connect(cameraWorker, &CameraWorker::finished, this, []() {
    qDebug() << "CameraWorker signaled finished";
    });

    cameraThread->start(); // 启动线程，但不立即开始处理数据
    qDebug() << "Camera thread initialized";
    enableflag = 2;

}

//使用相机线程更新中心点
void MainWindow::on_centerPointUpdated(float depth, float x, float y, float z) {
    QMutexLocker locker(&center_mutex); // Thread-safe access
    center_depth = depth;
    center_x = x;
    center_y = y;
    center_z = z;
//    qDebug() << "Center point updated: depth=" << depth << ", x=" << x << ", y=" << y << ", z=" << z;
}

//初始化某些参数
void MainWindow::init_parameters(){
    //机械臂末端坐标系下相机的旋转矩阵
    T_end_c << 0.7076, -0.707, 0.0009, 21.33,
                0.7067, 0.7076, -0.0062, -63.35,
                0.0037, 0.005, 0.99998, 23.74,
                0, 0, 0, 1;
    //初始化抓取起点
    outset.tran.x = 0;
    outset.tran.y = 0;
    outset.tran.z = 0;
    outset.rpy.rx = 0;
    outset.rpy.ry = 0;
    outset.rpy.rz = 0;



//    // 获取应用程序目录路径（即 build 目录），然后返回 .pro 文件所在的根目录路径
//    QString appDir = QCoreApplication::applicationDirPath();

//    // 如果路径包含 "build" 子目录，去掉它
//    QDir dir(appDir);
//    dir.cdUp(); // 返回上一级目录（即 .pro 所在目录）

//    // 拼接 weight 文件夹的路径
//    modelPath = dir.absolutePath().toStdString() + "/weights/best.onnx";
//    classNamesPath = dir.absolutePath().toStdString() + "/weights/class.names";
//    classNames = ::utils::loadNames(classNamesPath);
    QString weightsPath = getWeightPath();
    modelPath = weightsPath.toStdString() + "best.onnx";
    classNamesPath = weightsPath.toStdString() + "class.names";

    qDebug() << "Model path:" << QString::fromStdString(modelPath);
    qDebug() << "Class names path:" << QString::fromStdString(classNamesPath);

    classNames = ::utils::loadNames(classNamesPath);


}

QString MainWindow::getWeightPath() {
    QDir dir;
    QString executablePath = QCoreApplication::arguments().at(0);  // 获取启动命令的路径
    qDebug() << "Raw executable path (argv[0]):" << executablePath;

    // 如果是 AppImage 环境，解析符号链接到外部目录
    if (qgetenv("APPDIR").isEmpty()) {
        // 非 AppImage 环境，使用可执行文件目录并回溯
        dir = QDir(QCoreApplication::applicationDirPath());
        qDebug() << "Executable dir:" << dir.absolutePath();
        if (dir.absolutePath().contains("build")) {
            dir.cdUp();  // 从 build-DRobot-unknown-Release/ 到源目录
            qDebug() << "Adjusted dir to source:" << dir.absolutePath();
        }
    } else {
        // AppImage 环境，解析 argv[0] 的真实路径
        QFileInfo fileInfo(executablePath);
        QString realPath = fileInfo.canonicalFilePath();  // 解析符号链接
        qDebug() << "Canonical executable path:" << realPath;

        // 获取 AppImage 文件所在目录
        dir = QDir(realPath);
        dir.cdUp();  // 从 /path/to/Application-x86_64.AppImage 到 /path/to/
        qDebug() << "AppImage external dir:" << dir.absolutePath();
    }

    QString weightPath = dir.absolutePath() + "/weights/";
    qDebug() << "Final weights path:" << weightPath;

    if (!QDir(weightPath).exists()) {
        qDebug() << "Error: weights directory does not exist at" << weightPath;
    }

    return weightPath;
}

MainWindow::~MainWindow()
{
    store_setting_value();//保存上一次设置的参数
    if (cameraThread) {
            emit stopCamera(); // 停止数据处理
            cameraThread->quit();
            if (!cameraThread->wait(2000)) { // 等待退出
                qWarning() << "Camera thread did not exit, terminating";
                cameraThread->terminate();
                cameraThread->wait();
            }
        }

    delete ui;
    qDebug() << "MainWindow destructor completed";
}


void MainWindow::closeEvent(QCloseEvent *event)
{

    qDebug() << "Close event received";
    if (cameraThread) {
        emit stopCamera(); // 停止摄像头数据处理
        cameraThread->quit();
        if (!cameraThread->wait(2000)) {
            qWarning() << "Camera thread did not exit in closeEvent, terminating";
            cameraThread->terminate();
            cameraThread->wait();
        }
    }
    event->accept(); // 接受关闭事件
}

void MainWindow::get_setting_value(){
    QSettings settings("pulan", "Drobot");
    ui->store_x->setValue(settings.value("store_x", 0).toDouble());
    ui->store_y->setValue(settings.value("store_y", 0).toDouble());
    ui->store_z->setValue(settings.value("store_z", 0).toDouble());
    ui->store_rx->setValue(settings.value("store_rx", 0).toDouble());
    ui->store_ry->setValue(settings.value("store_ry", 0).toDouble());
    ui->store_rz->setValue(settings.value("store_rz", 0).toDouble());

    ui->JawX_SpinBox->setValue(settings.value("JawX_SpinBox", 0).toDouble());
    ui->JawY_SpinBox->setValue(settings.value("JawY_SpinBox", 0).toDouble());
    ui->JawZ_SpinBox->setValue(settings.value("JawZ_SpinBox", 0).toDouble());
}


void MainWindow::store_setting_value(){
    QSettings settings("pulan", "Drobot"); // 参数为组织名和应用名
    //储存上一次设定的存储角
    settings.setValue("store_x", ui->store_x->value());
    settings.setValue("store_y", ui->store_y->value());
    settings.setValue("store_z", ui->store_z->value());
    settings.setValue("store_rx", ui->store_rx->value());
    settings.setValue("store_ry", ui->store_ry->value());
    settings.setValue("store_rz", ui->store_rz->value());
    //存储上一次设定的夹爪参数
    settings.setValue("JawX_SpinBox", ui->JawX_SpinBox->value());
    settings.setValue("JawY_SpinBox", ui->JawY_SpinBox->value());
    settings.setValue("JawZ_SpinBox", ui->JawZ_SpinBox->value());

}



//从控件上的text文本中获取当前机械臂的各个参数
void MainWindow::Modify_MechArm_clicked()
{

    Jaw.x = ui->JawX_SpinBox->value()*1000;
    Jaw.y = ui->JawY_SpinBox->value()*1000;
    Jaw.z = ui->JawZ_SpinBox->value()*1000;


}



//将 OpenCV 的 cv::Mat 格式的图像显示到 Qt 的 QLabel 控件上
void LabelDisplayMat(QLabel *label, cv::Mat &mat)
{
    cv::Mat Rgb;
    QImage Img;
    if (mat.channels() == 3)//RGB Img
    {
        cv::cvtColor(mat, Rgb, cv::COLOR_BGR2RGB);//颜色空间转换
        Img = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.cols * Rgb.channels(), QImage::Format_RGB888);
    }
    else//Gray Img
    {
        Img = QImage((const uchar*)(mat.data), mat.cols, mat.rows, mat.cols*mat.channels(), QImage::Format_Indexed8);
    }
    // 显示图像到 QLabel
    label->setPixmap(QPixmap::fromImage(Img));
}






/// \param p_base_obj          目标位姿
/// \param pet          夹爪参数
/// \return             目标关节弧度
DescPose MainWindow::Tool_IK_DRobot(DescPose p_base_obj, DescTran pet)
{
    //将基坐标到目标的变换转换为旋转变换矩阵

    Eigen::Matrix4d T_base_obj = DescPosetoMatrix(p_base_obj);

    //将机械臂末端坐标到工具端末端的变换转换为旋转变换矩阵
    //x=0
    //y=0
    //z=
    Eigen::Matrix4d T_end_tool;
    T_end_tool.setIdentity();
    T_end_tool(0, 3) = pet.x;
    T_end_tool(1, 3) = pet.y;
    T_end_tool(2, 3) = pet.z;

    //求逆得到工具端末端到机械臂末端坐标的旋转变换矩阵
    Eigen::Matrix4d T_tool_end = T_end_tool.inverse();

    //基坐标到工具末端的旋转变换
    //最后实际就是夹爪的坐标和目标坐标重合所以这里有tar=grip
    //Base^tool_T*tool^End_T=Base^End_T
    Eigen::Matrix4d T_base_end = T_base_obj * T_tool_end;
    //用位姿表示旋转变换矩阵
    //求出机械臂末端应该到达的位置
    DescPose DP_base_end = matrixToDescPose(T_base_end);
    qDebug() << "DP_base_end x: " << DP_base_end.tran.x;
    qDebug() << "DP_base_end y: " << DP_base_end.tran.y;
    qDebug() << "DP_base_end z: " << DP_base_end.tran.z;

    return DP_base_end;


}



void MainWindow::on_ActArm_clicked()
{

    if (enableflag == 2)
    {

        Arm_Err =  robot.RPC("192.168.58.2"); //与 机 器 人 控 制 器 建 立 通 信 连 接
        if (Arm_Err == 0)
        {
            char version[64] = "";
            robot.GetSDKVersion(version);
            qDebug() << "version" << version;
            robot.RobotEnable(1);//机械臂使能
            sleep(1);
            robot.SetToolDO(0,0,0,0);
            robot.SetToolDO(1,0,0,0);
            ui->ArmState->setText("机械臂已连接");
            enableflag = 1;
            stateTime->start(100);
        }
        else {
            ui->ArmState->setText("机械臂未连接");
        }

    }
    if (enableflag == 0)
    {
        robot.RobotEnable(1);
        ui->ArmState->setText("手动模式");
        enableflag = 1;
        stateTime->start(50);
    }


}

void MainWindow::on_ActCamera_clicked() {
    //0表示初次启动，1表示正在运行，2表示相机被关闭
    if (cameraflag == 0 || cameraflag == 2) {
        emit startCamera();
        cameraflag = 1;
        qDebug() << "Camera started";
    } else {
        qDebug() << "Camera already active";
    }
}

void MainWindow::on_closeCam_clicked() {
    if (cameraflag == 1) {
        emit stopCamera();
        ui->Camera_current->clear();
        ui->PointCoord->clear();
        cameraflag = 2;
        qDebug() << "Camera stopped and GUI cleared";
    } else {
        qDebug() << "Camera not active";
    }
}

void MainWindow::statetimeout(){

    ui->SpeedValue->setText(QString::number(ui->AllSpeed->value())+"%");
    speed = ui->AllSpeed->value();
    robot.GetActualJointPosDegree(flag,&cur_angle_);
    robot.GetActualTCPPose(flag, &cur_pose_);
    vel = speed;

    ui->CurPosX->setText(QString::number(cur_pose_.tran.x,'f',2));
    ui->CurPosY->setText(QString::number(cur_pose_.tran.y,'f',2));
    ui->CurPosZ->setText(QString::number(cur_pose_.tran.z,'f',2));
    ui->CurPosRX->setText(QString::number(cur_pose_.rpy.rx,'f',2));
    ui->CurPosRY->setText(QString::number(cur_pose_.rpy.ry,'f',2));
    ui->CurPosRZ->setText(QString::number(cur_pose_.rpy.rz,'f',2));

    ui->CurJoint01->setText(QString::number(cur_angle_.jPos[0],'f',2));
    ui->CurJoint02->setText(QString::number(cur_angle_.jPos[1],'f',2));
    ui->CurJoint03->setText(QString::number(cur_angle_.jPos[2],'f',2));
    ui->CurJoint04->setText(QString::number(cur_angle_.jPos[3],'f',2));
    ui->CurJoint05->setText(QString::number(cur_angle_.jPos[4],'f',2));
    ui->CurJoint06->setText(QString::number(cur_angle_.jPos[5],'f',2));

    ui->joint1INFO->setText(QString::number(cur_angle_.jPos[0],'f',2));
    ui->joint2INFO->setText(QString::number(cur_angle_.jPos[1],'f',2));
    ui->joint3INFO->setText(QString::number(cur_angle_.jPos[2],'f',2));
    ui->joint4INFO->setText(QString::number(cur_angle_.jPos[3],'f',2));
    ui->joint5INFO->setText(QString::number(cur_angle_.jPos[4],'f',2));
    ui->joint6INFO->setText(QString::number(cur_angle_.jPos[5],'f',2));

    ui->x_data->setText(QString::number(cur_pose_.tran.x,'f',2));
    ui->y_data->setText(QString::number(cur_pose_.tran.y,'f',2));
    ui->z_data->setText(QString::number(cur_pose_.tran.z,'f',2));
    ui->roll_data->setText(QString::number(cur_pose_.rpy.rx,'f',2));
    ui->pitch_data->setText(QString::number(cur_pose_.rpy.ry,'f',2));
    ui->yaw_data->setText(QString::number(cur_pose_.rpy.rz,'f',2));

    //    storepoint.tran.x = ui->store_x->value();
    //    storepoint.tran.y = ui->store_y->value();
    //    storepoint.tran.z = ui->store_z->value();
    //    storepoint.rpy.rx = ui->store_rx->value();
    //    storepoint.rpy.ry = ui->store_ry->value();
    //    storepoint.rpy.rz = ui->store_rz->value();


}

void MainWindow::on_CloseArm_clicked()
{

    if(enableflag == 1)
    {
     ui->ArmState->setText("断开连接");
     robot.RobotEnable(0);//机械臂下使能
     stateTime->stop();
     enableflag = 0;
    }
}



//GUI界面更新起点
void MainWindow::updateoutset()
{
    ui->outset_x->setText(QString::number(outset.tran.x,'f',2));
    ui->outset_y->setText(QString::number(outset.tran.y,'f',2));
    ui->outset_z->setText(QString::number(outset.tran.z,'f',2));
    ui->outset_rx->setText(QString::number(outset.rpy.rx,'f',2));
    ui->outset_ry->setText(QString::number(outset.rpy.ry,'f',2));
    ui->outset_rz->setText(QString::number(outset.rpy.rz,'f',2));
}

//在GUI界面更新相机数据
void MainWindow::updateCameraFrame(const QImage& image, int frameRate) {
    ui->Camera_current->setPixmap(QPixmap::fromImage(image));
    ui->Camera_current->setFixedSize(640, 480);
    ui->Camera_current->setScaledContents(true);
    ui->framearg->setText(QString::number(frameRate) + "FPS");
}
//gui界面更新目标检测结果显示
void MainWindow::updatePoints(const QString& points) {
    ui->PointCoord->setText(points);
}



void MainWindow::on_MoveGrip_clicked() {
    // 检查是否需要显示弹窗
    if (showPopupEnabled) {
        // 创建自定义 QMessageBox
        QMessageBox msgBox(this);
        msgBox.setWindowTitle("确认操作");
        msgBox.setText("是否正确设置存储点位姿？");

        // 添加确认和取消按钮
        QPushButton *confirmButton = msgBox.addButton("是", QMessageBox::AcceptRole);
        msgBox.addButton("否", QMessageBox::RejectRole);

        // 添加复选框
        QCheckBox *checkBox = new QCheckBox("下次不再提示", &msgBox);
        msgBox.setCheckBox(checkBox);

        // 显示弹窗并等待用户操作
        msgBox.exec();
        // 检查复选框状态并更新 showPopupEnabled
        if (checkBox->isChecked()) {
            showPopupEnabled = false; // 下次不再显示弹窗
        }
        // 检查用户是否点击了“确认”
        if (msgBox.clickedButton() == confirmButton) {

            startmove();

        } else {
            return; // 用户取消，直接返回
        }
    } else {
        startmove();

    }
}
//移动到距离相机最近的目标上方
void MainWindow::startmove()
{

    if (points_obj.size()!=0){


        //在连续抓取中，第n次中检测到物体了，我就不用再触发这个函数
        //假如我回到起点了，但是没有检测到物体，则我需要继续检测抓取
        //一开始我没有启动这个定时器则不需要操作
        if (fiveTime->isActive()) {
                fiveTime->stop(); // 停止定时器
            }

        std::vector<Eigen::Vector4d> P_c_obj_list; // 存储所有点的相机坐标
        Eigen::Vector4d P_base_obj;   // 基坐标系下的点
        Eigen::Matrix4d T_base_end = DescPosetoMatrix(cur_pose_);
        Eigen::Matrix4d T_base_c = T_base_end * T_end_c;
        // 加锁以安全访问 points_obj
        mutex.lock();
        P_c_obj_list.resize(points_obj.size());
        for (int i = 0; i < points_obj.size(); ++i) {
            P_c_obj_list[i][0] = points_obj[i].x ;
            P_c_obj_list[i][1] = points_obj[i].y ;
            P_c_obj_list[i][2] = points_obj[i].z ;
            P_c_obj_list[i][3] = 1.0; // 齐次坐标
        }
        mutex.unlock();
        int min_idx = 0;
        // 找到 X、Y 偏移量最小的目标点

        float min_xy_offset = std::numeric_limits<float>::max();
        for (int i = 0; i < static_cast<int>(P_c_obj_list.size()); ++i) {
            float xy_offset = fabs(P_c_obj_list[i][0]) + fabs(P_c_obj_list[i][1]); // 使用 |x| + |y| 简化计算
            // 或者使用欧几里得距离：
            //float xy_offset = sqrt(P_c_obj_list[i][0] * P_c_obj_list[i][0] + P_c_obj_list[i][1] * P_c_obj_list[i][1]);
            if (xy_offset < min_xy_offset) {
                min_xy_offset = xy_offset;
                min_idx = i;
            }
        }
        qDebug() << "min_idx : " << min_idx;

        P_base_obj = T_base_c * P_c_obj_list[min_idx];
        //获取目标点上方300mm的位置，姿态沿用当前姿态
        //
        DescPose tarPT = cur_pose_;
        outset = cur_pose_;//设置起点
        updateoutset();//更新GUI界面的显示
        tarPT.tran.x = P_base_obj[0];
        tarPT.tran.y = P_base_obj[1];
        tarPT.tran.z = P_base_obj[2]+250;
        tarPT.rpy.rx = 180;
        tarPT.rpy.ry = 0;
        //相机下的目标点
        qDebug() << "P_base_obj : " << P_base_obj[1];

        //tar_robotend机械臂末端应到达位置
        tar_robotend = Tool_IK_DRobot(tarPT,Jaw);//求解夹爪到达目标位姿
        //前往目标上方
        Arm_Err = robot.MoveCart(&tar_robotend,tool,user,speed,acc,ovl,blendT,-1);
        if( Arm_Err != 0)
        {
            ui->ArmState->setText("错误码："+ QString::number(Arm_Err));
            return;
        }
        else{
            zeroTime->start(300);
        }


    }
    else{
        ui->PointCoord->clear();
        ui->PointCoord->setText("暂时未检测到目标！");
    }
}

//移动到目标处
void MainWindow::zeroTimeout(){
    uint8_t state = 0;
    robot.GetRobotMotionDone(&state);
    if (state != 0) {// 运动完成
        zeroTime->stop();
        //等待2s让相机重新识别
        //否则在机械臂移动过程中被识别保存的坐标会造成错误
        sleep_cp(2000);
        while(1){
            if (points_obj.size()!=0){

                std::vector<Eigen::Vector4d> P_c_obj_list; // 存储所有点的相机坐标
                Eigen::Vector4d P_base_obj;   // 基坐标系下的点
                Eigen::Matrix4d T_base_end = DescPosetoMatrix(cur_pose_);
                Eigen::Matrix4d T_base_c = T_base_end * T_end_c;
                // 加锁以安全访问 points_obj
                mutex.lock();
                P_c_obj_list.resize(points_obj.size());
                for (int i = 0; i < points_obj.size(); ++i) {
                    P_c_obj_list[i][0] = points_obj[i].x ;//加上自定义补偿，一般设为0
                    P_c_obj_list[i][1] = points_obj[i].y ;
                    P_c_obj_list[i][2] = points_obj[i].z ;
                    P_c_obj_list[i][3] = 1.0; // 齐次坐标
                }
                mutex.unlock();
                P_base_obj = T_base_c * P_c_obj_list[0];
                DescPose tarPT = cur_pose_;
                tarPT.tran.x = P_base_obj[0];
                tarPT.tran.y = P_base_obj[1];
                tarPT.tran.z = P_base_obj[2];
                tarPT.rpy.rx = 180;
                tarPT.rpy.ry = 0;
                //tar_robotend机械臂末端应到达位置
                tar_robotend = Tool_IK_DRobot(tarPT,Jaw);//求解夹爪到达目标位姿
                JointPos tar_angel = {0,0,0,0,0,0};

                Arm_Err = robot.GetInverseKin(0,&tar_robotend,-1,&tar_angel);
                if( Arm_Err != 0)
                {
                    ui->PointCoord->clear();
                    ui->PointCoord->setText("错误码："+ QString::number(Arm_Err));
                    return;
                }
                //前往目标上方
                DescPose offset_pos;
                memset(&offset_pos, 0, sizeof(DescPose));
                memset(&epos, 0, sizeof(ExaxisPos));
                robot.SetSpeed(speed);
                int Arm_Err = robot.MoveL(&tar_angel, &tar_robotend, tool, user, vel, acc, ovl, blendR, &epos, search,flag, &offset_pos);
                //Arm_Err = robot.MoveL(&tar_robotend,tool,user,speed,acc,ovl,blendT,-1);
                if( Arm_Err != 0)
                {
                    ui->ArmState->setText("错误码："+ QString::number(Arm_Err));
                    return;
                }

                FirstTime->start(300);
                break; // 成功抓取后跳出循环
                }

        }
    }
    else {
        qDebug() << "Robot still moving to obj upsize";
    }

}

//抓取目标并向上提250mm
void MainWindow::FirstTimeout(){
    uint8_t state = 0;
    robot.GetRobotMotionDone(&state);
    if (state != 0) { // 运动完成
        FirstTime->stop();
        qDebug() << "Robot reached target position";
        FirstTime->stop();
        on_CloseGrip_clicked();//闭合夹爪

        // 向上移动 350 mm
        DescPose current_pose;
        robot.GetActualTCPPose(flag, &current_pose);
        DescPose lift_pose = current_pose;
        lift_pose.tran.z += 250; // 增加 250 mm (0.25 m)

        errno_t Arm_Err =  robot.MoveCart(&lift_pose, tool, user, speed, acc, ovl, blendT, -1);
        if (Arm_Err != 0)
        {
            lift_pose.tran.z -= 150;
            Arm_Err =  robot.MoveCart(&lift_pose, tool, user, speed, acc, ovl, blendT, -1);
            if(Arm_Err != 0)
            {
                ui->ArmState->setText("错误码："+ QString::number(Arm_Err));
                return;
            }
        }
        //根据用户需求是否前往储存点
        if (ui->isstore->isChecked() || ui->isoutset->isChecked() )
        {
            ui->isstore->setChecked(true); // 强制勾选 isstore
            SenTime->start(300);
        }

    }
    else {
        qDebug() << "Robot still moving to obj! ";
    }


}

//移动到存储点
void MainWindow::SenTimeout(){
    uint8_t state = 0;
    robot.GetRobotMotionDone(&state);
    if (state != 0) { // 运动完成
        SenTime->stop();
        //获取设定的存储点
        //设定最终姿态，因为保证最后的姿态一定是定的
        storepoint.tran.x = ui->store_x->value();
        storepoint.tran.y = ui->store_y->value();
        storepoint.tran.z = ui->store_z->value();
        storepoint.rpy.rx = 180;
        storepoint.rpy.ry = 0;
        storepoint.rpy.rz = ui->store_rz->value();


        errno_t Arm_Err =  robot.MoveCart(&storepoint, tool, user, speed, acc, ovl, blendT, -1);
        if (Arm_Err != 0)
        {
             ui->ArmState->setText("错误码："+ QString::number(Arm_Err));
            return;

        }
        triTime->start(300);//判断是否到达，同时打开夹爪

    }
}


void MainWindow::TriTimeout(){
    uint8_t state = 0;
    robot.GetRobotMotionDone(&state);
    if (state != 0) { // 运动完成
        triTime->stop();
        sleep_cp(2000);
        // Use the center point data from the camera thread
        Eigen::Vector4d p_c_obj;

        {
            QMutexLocker locker(&center_mutex); // Thread-safe access
            p_c_obj[0] = center_x*1000;
            p_c_obj[1] = center_y*1000;
            p_c_obj[2] = center_z*1000;//单位mm
            p_c_obj[3] = 1;

        }


        //此时获取了目标存点但是不能直接放，因为机械臂得到的是地面的点
        //要机械臂末端到这个点需要的一定距离，由于是垂直的所以需要测量夹爪加上木块总长L
        //这个值一般不固定，这是由于夹爪的抓取精度问题导致的
        //但是误差应该不超过1cm，所以我们每次测量就在原来的基础上再加上temp=8mm作为缓冲


        //计算相机与机械臂末端的旋转变换，计算机械臂末端此时距离地面的距离h和对应的点T_end_obj
        //然后移动h-L-temp,在加上原来的点（x,y）就是目标点

        Eigen::Vector4d p_end_obj = T_end_c * p_c_obj;
        qDebug() << "p_end_obj[x]:" << p_end_obj[0];
        qDebug() << "p_end_obj[y]:" << p_end_obj[1];
        qDebug() << "p_end_obj[z]:" << p_end_obj[2];

        p_end_obj[2] = p_end_obj[2] - 8 - 210 - blocklength*num;//这里的210是夹爪加上积木的长度
        Eigen::Matrix4d T_base_end = DescPosetoMatrix(cur_pose_);
        Eigen::Vector4d p_base_obj = T_base_end * p_end_obj;

        //求解需要的关节角
        //p_end_obj机械臂末端距离地面高度
        DescPose tarPT = cur_pose_;
        tarPT.tran.z = p_base_obj[2];
        qDebug() << "tarPT.tran.x " << tarPT.tran.x;
        qDebug() << "tarPT.tran.y " << tarPT.tran.y;
        qDebug() << "tarPT.tran.z " << tarPT.tran.z;
        JointPos tar_angel = {0,0,0,0,0,0};

        Arm_Err = robot.GetInverseKin(0,&tarPT,-1,&tar_angel);
        if( Arm_Err != 0)
        {
            ui->PointCoord->clear();
            ui->PointCoord->setText("错误码："+ QString::number(Arm_Err));
            return;
        }
        DescPose offset_pos;
        memset(&offset_pos, 0, sizeof(DescPose));
        memset(&epos, 0, sizeof(ExaxisPos));
        robot.SetSpeed(speed);
        int Arm_Err = robot.MoveL(&tar_angel, &tarPT, tool, user, vel, acc, ovl, blendR, &epos, search,flag, &offset_pos);

        if( Arm_Err != 0)
        {
            ui->ArmState->setText("错误码："+ QString::number(Arm_Err));
            return;
        }
        num++;//积木数+1
        fourTime->start(300);




//        on_OpenGrip_clicked();//打开夹爪
        //根据用户需求是否返回起点
        //假如选择了连续抓取，则此时一定会储存和返回起点
//        if (ui->isoutset->isChecked() )
//        {
//            //回到起点
//            errno_t Arm_Err =  robot.MoveCart(&outset, tool, user, speed, acc, ovl, blendT, -1);
//            if (Arm_Err != 0)
//            {
//                ui->ArmState->setText("错误代码："+ QString::number(Arm_Err));
//                return;

//            }
//            fourTime->start(300);

//        }
    }
    else
    {
        qDebug() << "Robot still moving to 储存点 ";
    }
}

void MainWindow::FourTimeout(){
    uint8_t state = 0;
    robot.GetRobotMotionDone(&state);
    if (state != 0) { // 运动完成
        fourTime->stop();
        on_OpenGrip_clicked();//打开夹爪
        DescPose tarPT = cur_pose_;
        tarPT.tran.z = tarPT.tran.z + 110 ;//上升110mm避免碰到积木
        JointPos tar_angel = {0,0,0,0,0,0};

        Arm_Err = robot.GetInverseKin(0,&tarPT,-1,&tar_angel);
        if( Arm_Err != 0)
        {
            ui->PointCoord->clear();
            ui->PointCoord->setText("错误码："+ QString::number(Arm_Err));
            return;
        }
        DescPose offset_pos;
        memset(&offset_pos, 0, sizeof(DescPose));
        memset(&epos, 0, sizeof(ExaxisPos));
        robot.SetSpeed(speed);
        int Arm_Err = robot.MoveL(&tar_angel, &tarPT, tool, user, vel, acc, ovl, blendR, &epos, search,flag, &offset_pos);
        if( Arm_Err != 0)
        {
            ui->PointCoord->clear();
            ui->PointCoord->setText("错误码："+ QString::number(Arm_Err));
            return;
        }

    }
    else{
        qDebug() << "return outseting! ";
    }

}


void MainWindow::on_StorePos_clicked()
{
    //获取设定的存储点
    storepoint.tran.x = ui->store_x->value();
    storepoint.tran.y = ui->store_y->value();
    storepoint.tran.z = ui->store_z->value();
    storepoint.rpy.rx = ui->store_rx->value();
    storepoint.rpy.ry = ui->store_ry->value();
    storepoint.rpy.rz = ui->store_rz->value();

    errno_t Arm_Err =  robot.MoveCart(&storepoint, tool, user, speed, acc, ovl, blendT, -1);
    if (Arm_Err != 0)
    {

        ui->ArmState->setText("错误代码："+ QString::number(Arm_Err));

    }




}




void MainWindow::on_modeswitch_clicked()
{

    uint8_t state;
    robot.IsInDragTeach(&state);
    if (enableflag == 1 )
    {
        if (state == false)//如果不是拖动模式
        {
           robot.Mode(1);//手动模式
           sleep(1);
           robot.DragTeachSwitch(1);//切换拖动模式
           sleep(1);
           ui->ArmState->setText("拖动模式");
           ui->ArgSetPush->hide();

           ui->ControlPush->hide();
           ui->TeachCon->hide();
           ui->ActArm->hide();
           ui->ActCamera->hide();
           ui->CloseArm->hide();
           ui->CloseGrip->hide();
           ui->MoveGrip->hide();
           ui->MoveGripCon->hide();
           ui->OpenGrip->hide();
           ui->PointCoord->hide();
           ui->Idle_Pos->hide();
           ui->StorePos->hide();
           ui->back_outset->hide();
           ui->closeCam->hide();
           ui->stop->hide();

        }
        else if (state == true)
        {
            robot.Mode(1);//手动模式
            sleep(1);
            robot.DragTeachSwitch(0);//非切换拖动模式
//            sleep(1);
//            robot.Mode(0);//自动模式
            ui->ArmState->setText("自动模式");
            ui->ArgSetPush->show();
            ui->ControlPush->show();
            ui->TeachCon->show();
            ui->ActArm->show();
            ui->ActCamera->show();
            ui->CloseArm->show();
            ui->CloseGrip->show();
            ui->MoveGrip->show();
            ui->MoveGripCon->show();
            ui->OpenGrip->show();
            ui->PointCoord->show();
            ui->Idle_Pos->show();
            ui->StorePos->show();
            ui->back_outset->show();
            ui->closeCam->show();
            ui->stop->show();

        }
    }
    else
    {
        ui->ArmState->setText("机械臂未连接！");

    }


}




void MainWindow::on_CloseGrip_clicked()
{

     ui->gripstate->clear();
    if (enableflag == 1)
    {
        robot.SetToolDO(0,0,0,0);
        robot.SetToolDO(1,0,0,0);
        robot.SetToolDO(1, 1, 0, 0);
        sleep(1);
        robot.SetToolDO(0,0,0,0);
        robot.SetToolDO(1,0,0,0);
    }
    else
    {
        ui->gripstate->setText("请先激活机械臂");
    }
}

void MainWindow::on_OpenGrip_clicked()
{
     ui->gripstate->clear();
    if (enableflag == 1)
    {

        robot.SetToolDO(0,0,0,0);
        robot.SetToolDO(1,0,0,0);
        robot.SetToolDO(0, 1, 0, 0);
        sleep(1);
        robot.SetToolDO(0,0,0,0);
        robot.SetToolDO(1,0,0,0);
    }
    else
    {
        ui->gripstate->setText("请先激活机械臂");
    }
}

void MainWindow::on_ArgSetPush_clicked()
{
    if(ui->ArgInput->isVisible()){
        ui->ArgInput->hide();ui->JointCon->hide();
    }
    else{
        ui->ArgInput->show();ui->JointCon->hide();
    }
}



void MainWindow::on_TeachCon_clicked()
{
    if(ui->JointCon->isVisible()){
        ui->ArgInput->hide();ui->JointCon->hide();
    }
    else{
        ui->ArgInput->hide();ui->JointCon->show();
    }
}

void MainWindow::on_ControlPush_clicked()
{
    if(ui->ControlShow->isHidden()){
        ui->ControlShow->show();
    }
    else{
        ui->ControlShow->hide();
    }
}



void MainWindow::on_PosDes_clicked()
{
    DescPose a;
    a.tran.x = ui->XEdit->text().toFloat();
    a.tran.y = ui->YEdit->text().toFloat();
    a.tran.z = ui->ZEdit->text().toFloat();
    a.rpy.rx = ui->RXEdit->text().toFloat();
    a.rpy.ry = ui->RYEdit->text().toFloat();
    a.rpy.rz = ui->RZEdit->text().toFloat();

    Arm_Err = robot.MoveCart(&a,tool,user,speed,acc,ovl,blendT,-1);
    if (Arm_Err != 0)
    {

        ui->ArmState->setText("错误代码："+ QString::number(Arm_Err));

    }

}



void MainWindow::on_JointDes_clicked()
{
    if (enableflag == 1)
    {
        JointPos a;
        a.jPos[0] = ui->Joint1Edit->text().toFloat();
        a.jPos[1] = ui->Joint2Edit->text().toFloat();
        a.jPos[2] = ui->Joint3Edit->text().toFloat();
        a.jPos[3] = ui->Joint4Edit->text().toFloat();
        a.jPos[4] = ui->Joint5Edit->text().toFloat();
        a.jPos[5] = ui->Joint6Edit->text().toFloat();
        DescPose temp;

        robot.GetForwardKin(&a, &temp);
        robot.MoveCart(&temp,tool,user,speed,acc,ovl,blendT,-1);

    }
    else
    {
        ui->gripstate->setText("请先激活机械臂");
    }


}


void MainWindow::on_MoveGripCon_clicked()
{
    if (congrip == false)
    {
        congrip = true;
        ui->isstore->setChecked(true); // 强制勾选 isstore
        ui->isoutset->setChecked(true);//强制勾选 isoutset
        // 设置红色背景，保留边框和文字颜色
        ui->MoveGripCon->setStyleSheet("QPushButton { background-color: red; color: white; border: 1px solid gray; }");
        on_MoveGrip_clicked();
    }
    else
    {
        ui->MoveGripCon->setStyleSheet(""); // 恢复默认颜色
        congrip = false;

    }

}



void MainWindow::on_stop_clicked()
{
    //急停机械臂

    robot.PauseMotion();
    robot.StopMotion();
}


void MainWindow::on_Set_storepoint_clicked()
{
    Arm_Err = robot.GetActualToolFlangePose(1, &storepoint);
    if( Arm_Err != 0)
    {
        ui->PointCoord->clear();
        ui->PointCoord->setText("错误代码："+ QString::number(Arm_Err));

    }
    else
    {

        ui->store_x->setValue(storepoint.tran.x);
        ui->store_y->setValue(storepoint.tran.y);
        ui->store_z->setValue(storepoint.tran.z);
        ui->store_rx->setValue(storepoint.rpy.rx);
        ui->store_ry->setValue(storepoint.rpy.ry);
        ui->store_rz->setValue(storepoint.rpy.rz);

    }
}




void MainWindow::on_Idle_Pos_clicked()
{
    if(enableflag == 1){
        JointPos j1;
        DescPose desc_pos1,offset_pos;
        ExaxisPos  epos;

        memset(&j1, 0, sizeof(JointPos));
        memset(&desc_pos1, 0, sizeof(DescPose));
        memset(&epos, 0, sizeof(ExaxisPos));

        j1 = {0,-90,0,-90,1,-45};
        Arm_Err = robot.GetForwardKin(&j1,&desc_pos1);
        if (Arm_Err != 0)
        {
            ui->ArmState->setText("错误码:" + QString::number(Arm_Err));

        }

        Arm_Err = robot.MoveJ(&j1, &desc_pos1, tool, user, speed, acc, ovl, &epos, blendT,flag, &offset_pos);
        if (Arm_Err != 0)
        {
           ui->ArmState->setText("错误码:" + QString::number(Arm_Err));

        }
    }
    else{
        ui->ArmState->setText("机械臂未连接");
    }


}

void MainWindow::on_back_outset_clicked()
{
    //回到起点
    //判断起点是否被设置
    if (isDescPoseZero(outset))
    {
        ui->ArmState->setText("起点未正确设置！");
    }
    else
    {
        errno_t Arm_Err =  robot.MoveCart(&outset, tool, user, speed, acc, ovl, blendT, -1);
        if (Arm_Err != 0)
        {
            ui->ArmState->setText("错误码:" + QString::number(Arm_Err));
            return;

        }

    }


}

bool MainWindow::isDescPoseZero(const DescPose& pose, float epsilon) {
    return (fabs(pose.tran.x) < epsilon &&
            fabs(pose.tran.y) < epsilon &&
            fabs(pose.tran.z) < epsilon &&
            fabs(pose.rpy.rx) < epsilon &&
            fabs(pose.rpy.ry) < epsilon &&
            fabs(pose.rpy.rz) < epsilon);
}

void MainWindow::on_joint1TeachDes_pressed()
{
    uint8_t num = 1; uint8_t direction = 0;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint1TeachInc_pressed()
{
    uint8_t num = 1; uint8_t direction = 1;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint2TeachDes_pressed()
{
    uint8_t num = 2; uint8_t direction = 0;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint2TeachInc_pressed()
{
    uint8_t num = 2; uint8_t direction = 1;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint3TeachDes_pressed()
{
    uint8_t num = 3; uint8_t direction = 0;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint3TeachInc_pressed()
{
    uint8_t num = 3; uint8_t direction = 1;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint4TeachDes_pressed()
{
    uint8_t num = 4; uint8_t direction = 0;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint4TeachInc_pressed()
{
    uint8_t num = 4; uint8_t direction = 1;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint5TeachDes_pressed()
{
    uint8_t num = 5; uint8_t direction = 0;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint5TeachInc_pressed()
{
    uint8_t num = 5; uint8_t direction = 1;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint6TeachDes_pressed()
{
    uint8_t num = 6; uint8_t direction = 0;
    robot.StartJOG(0,num,direction,speed,acc,30);
}
void MainWindow::on_joint6TeachInc_pressed()
{
    uint8_t num = 6; uint8_t direction = 1;
    robot.StartJOG(0,num,direction,speed,acc,30);
}

void MainWindow::on_joint1TeachInc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint1TeachDes_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint2TeachDes_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint2TeachInc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint3TeachDes_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint3TeachInc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint4TeachDes_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint4TeachInc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint5TeachDes_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint5TeachInc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint6TeachDes_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_joint6TeachInc_released()
{
    robot.StopJOG(1);//减速停止
}

void MainWindow::on_pos_x_inc_pressed()
{
    uint8_t num = 1; uint8_t direction = 1;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_pos_x_des_pressed()
{
    uint8_t num = 1; uint8_t direction = 0;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_pos_y_inc_pressed()
{
    uint8_t num = 2; uint8_t direction = 1;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_pos_y_des_pressed()
{
    uint8_t num = 2; uint8_t direction = 0;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_pos_z_inc_pressed()
{
    uint8_t num = 3; uint8_t direction = 1;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_pos_z_des_pressed()
{
    uint8_t num = 3; uint8_t direction = 0;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_rollx_inc_pressed()
{
    uint8_t num = 4; uint8_t direction = 1;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_rollx_des_pressed()
{
    uint8_t num = 4; uint8_t direction = 0;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_pitchy_inc_pressed()
{
    uint8_t num = 5; uint8_t direction = 1;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_pitchy_des_pressed()
{
    uint8_t num = 5; uint8_t direction = 0;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_yawz_inc_pressed()
{
    uint8_t num = 6; uint8_t direction = 1;
    robot.StartJOG(2,num,direction,speed,acc,30);
}
void MainWindow::on_yawz_des_pressed()
{
    uint8_t num = 6; uint8_t direction = 0;
    robot.StartJOG(2,num,direction,speed,acc,30);
}

void MainWindow::on_pos_x_inc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_pos_x_des_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_pos_y_inc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_pos_y_des_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_pos_z_inc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_pos_z_des_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_rollx_inc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_rollx_des_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_pitchy_inc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_pitchy_des_released()
{
    robot.StopJOG(1);//减速停止;
}
void MainWindow::on_yawz_inc_released()
{
    robot.StopJOG(1);//减速停止
}
void MainWindow::on_yawz_des_released()
{
    robot.StopJOG(1);//减速停止
}


