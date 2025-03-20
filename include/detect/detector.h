#pragma once
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <utility>

#include "utils.h"


class YOLODetector
{
public:
    // 构造函数，接收一个 nullptr_t 类型的参数，仅用于特定的情况
    explicit YOLODetector(std::nullptr_t) {};

    // 构造函数，接收模型路径、是否使用GPU、输入图像大小作为参数，初始化 YOLO 模型检测器
    YOLODetector(const std::string& modelPath, 
                 const bool& isGPU, 
                 const cv::Size& inputSize);

    // 图像检测函数，使用给定的图片和阈值进行 YOLO 检测
    // 返回检测结果的检测对象列表
    std::vector<Detection> detect(cv::Mat &image, const float& confThreshold, const float& iouThreshold);

private:
    // ONNX 环境
    Ort::Env env{nullptr};
    
    // ONNX 会话选项
    Ort::SessionOptions sessionOptions{nullptr};
    
    // ONNX 会话，用于加载和运行模型
    Ort::Session session{nullptr};

    // 数据预处理函数，将图像转换为适合模型输入的格式
    // 将图像转换为输入张量所需的 Blob 格式，并返回输入张量的形状
    void preprocessing(cv::Mat &image, float*& blob, std::vector<int64_t>& inputTensorShape);

    // 后处理函数，处理模型输出，解析出目标检测框和类别
    // 根据输出的张量、置信度和 IOU 阈值过滤检测结果
    std::vector<Detection> postprocessing(const cv::Size& resizedImageShape, 
                                          const cv::Size& originalImageShape,
                                          std::vector<Ort::Value>& outputTensors, 
                                          const float& confThreshold, 
                                          const float& iouThreshold);

    // 辅助函数，用于从一个浮点数数组中获取最好的检测类别信息
    // 输入一个包含检测置信度的浮点数迭代器，返回置信度最高的类别ID和置信度
    static void getBestClassInfo(std::vector<float>::iterator it, const int& numClasses, 
                                 float& bestConf, int& bestClassId);

    // 是否使用动态输入形状的标志
    bool isDynamicInputShape{};

    // 输入图像的大小（宽和高）
    cv::Size2f inputImageShape;

    // 输入节点名称（ONNX 模型输入的名称）
    std::vector<Ort::AllocatedStringPtr> inputNodeNameAllocatedStrings;
    std::vector<const char*> inputNames;

    // 输出节点名称（ONNX 模型输出的名称）
    std::vector<Ort::AllocatedStringPtr> outputNodeNameAllocatedStrings;
    std::vector<const char*> outputNames;
};
