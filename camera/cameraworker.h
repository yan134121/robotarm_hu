#ifndef CAMERAWORKER_H
#define CAMERAWORKER_H

#include <QObject>
#include <QTimer>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include "detect/detector.h"
#include <QMutex>
#include <QDateTime>
#include <QImage>
#include "detect/utils.h"
#include "robot.h"
#include "robot_error.h"
#include "robot_types.h"
class CameraWorker : public QObject {
    Q_OBJECT
public:
    CameraWorker(const std::string& modelPath, const std::vector<std::string>& classNames,float confThreshold,
    float iouThreshold,QList<DescTran>& pointsObj, QMutex& mutex, bool& detectJ, QObject* parent = nullptr);
    ~CameraWorker();

public slots:
    void startCamera();
    void stopCamera();
    void test_vis();

signals:
    void frameReady(const QImage& image, int frameRate);
    void pointsUpdated(const QString& points);
    void finished(); // 用于线程完全退出时通知
    void centerPointUpdated(float depth, float x, float y, float z); // New signal for center point data

private:
    rs2::depth_frame getAlignedImages();
    void calculateCenterPoint(); // New method to calculate center point data

    QTimer* timer;
    rs2::pipeline pipeDR;
    rs2::config cfg;
    rs2::align ali = RS2_STREAM_COLOR;
    YOLODetector* detector;
    std::string modelPath;
    std::vector<std::string> classNames;
    QList<DescTran>& points_obj; 
    float confThreshold;  // 检测的置信度阈值
    float iouThreshold;   // NMS（非极大值抑制）的IOU阈值
    bool& detectJ;
    QMutex& mutex;
    cv::Mat color_image_m, depth_image_m;
    rs2_intrinsics intr, depth_intrin;
    int frameCnt = 0;
    QDateTime ct;
    bool isRunning = false; // 控制数据处理状态
};

#endif // CAMERAWORKER_H
