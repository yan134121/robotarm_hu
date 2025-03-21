#include "cameraworker.h"
#include <QDebug>
#include <QString>

CameraWorker::CameraWorker(const std::string& modelPath, const std::vector<std::string>& classNames,const float confThreshold,
                           const float iouThreshold,QList<DescTran>& pointsObj, QMutex& mutex, bool& detectJ, QObject* parent)
    : QObject(parent),timer(new QTimer(this)), modelPath(modelPath), classNames(classNames), points_obj(pointsObj),confThreshold(confThreshold),
      iouThreshold(iouThreshold), detectJ(detectJ), mutex(mutex), detector(nullptr), isRunning(false) {
    connect(timer, &QTimer::timeout, this, &CameraWorker::test_vis);
    qDebug() << "CameraWorker created";
}

CameraWorker::~CameraWorker() {
    stopCamera(); // 确保停止数据处理
    delete detector;
    qDebug() << "CameraWorker destroyed";
}

void CameraWorker::startCamera() {
    if (isRunning) {
        qDebug() << "CameraWorker already running";
        return;
    }

    if (!detector) { // 仅在第一次启动时初始化
        try {
            detector = new YOLODetector(modelPath, 0, cv::Size(640, 640));
            qDebug() << "Model was initialized.";
        } catch (const std::exception& e) {
            qWarning() << "Exception:" << e.what();
            delete detector;
            detector = nullptr;
            emit finished();
            return;
        }
    }

    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 6);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 6);
    pipeDR.start(cfg);
    timer->start(166);
    isRunning = true;
    qDebug() << "CameraWorker started";
}

void CameraWorker::stopCamera() {
    if (!isRunning) {
        qDebug() << "CameraWorker not running";
        return;
    }

    timer->stop();
    try {
        pipeDR.stop();
        qDebug() << "RealSense pipeline stopped";
    } catch (const std::exception& e) {
        qWarning() << "Error stopping pipeline:" << e.what();
    }
    isRunning = false;
    qDebug() << "CameraWorker stopped (data processing)";
}

rs2::depth_frame CameraWorker::getAlignedImages() {
    rs2::frameset frames = pipeDR.wait_for_frames();
    rs2::frameset aligned_frames = ali.process(frames);
    rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
    rs2::video_frame color_frame = aligned_frames.get_color_frame();

    intr = color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    depth_intrin = aligned_depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    cv::Mat depth_image(cv::Size(intr.width, intr.height), CV_16UC1, (void*)aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
    depth_image_m = depth_image;
    cv::Mat color_image(cv::Size(intr.width, intr.height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    color_image_m = color_image;

    return aligned_depth_frame;
}

void CameraWorker::test_vis() {
    if (!isRunning) return;

    rs2::depth_frame de_f = getAlignedImages();

    std::vector<Detection> result;
    ct = QDateTime::currentDateTime();
    if (detectJ && detector) {
        result = detector->detect(color_image_m, confThreshold, iouThreshold);
        ::utils::visualizeDetection(color_image_m, result, classNames);
    }

    int tt = ct.msecsTo(QDateTime::currentDateTime());
    if (tt > 0) frameCnt = int(1.0 / (tt * 0.001));
    else frameCnt = 0;

    cv::cvtColor(color_image_m, color_image_m, cv::COLOR_BGR2RGB);
    QImage image((const unsigned char*)(color_image_m.data), color_image_m.cols, color_image_m.rows, QImage::Format_RGB888);
    emit frameReady(image, frameCnt);


    // 计算中心点
        calculateCenterPoint();

    if (!detectJ) return;

    QString points;
    mutex.lock();
    points_obj.clear();
    for (size_t i = 0; i < result.size(); ++i) {
        cv::Point center(result[i].box.x + result[i].box.width / 2, result[i].box.y + result[i].box.height / 2);
        uint16_t depth_value = depth_image_m.at<uint16_t>(center.y, center.x);
        float dis = depth_value * 0.001f;
        float pixel[2] = { (float)center.x, (float)center.y };
        float point[3];
        rs2_deproject_pixel_to_point(point, &intr, pixel, dis);

        points += QString("第%1个的坐标：\n(%2, %3, %4)\n")
                     .arg(i).arg(point[0], 0, 'f', 3).arg(point[1], 0, 'f', 3).arg(point[2], 0, 'f', 3);

        DescTran point_obj;
        point_obj.x = point[0] * 1000;
        point_obj.y = point[1] * 1000;
        point_obj.z = point[2] * 1000;
        points_obj.append(point_obj);
    }
    mutex.unlock();

    emit pointsUpdated(points);
}

void CameraWorker::calculateCenterPoint() {
    if (depth_image_m.empty()) return;

    // Get the center of the depth frame
    int center_x = depth_image_m.cols / 2;
    int center_y = depth_image_m.rows / 2;


    // Get depth value at the center
    uint16_t depth_value = depth_image_m.at<uint16_t>(center_y, center_x);
    float depth = depth_value * 0.001f; // Convert to meters

    // Deproject pixel to 3D point in camera coordinate system
    float pixel[2] = {(float)center_x, (float)center_y};
    float point[3];
    rs2_deproject_pixel_to_point(point, &intr, pixel, depth);

    // Emit the center point data (depth and 3D coordinates)
    emit centerPointUpdated(depth, point[0], point[1], point[2]);
}
