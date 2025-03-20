#include "detect/detector.h"

YOLODetector::YOLODetector(const std::string& modelPath,
                           const bool& isGPU = true,
                           const cv::Size& inputSize = cv::Size(640, 640))
{
    // 初始化 ONNX 环境，日志等级为警告
    env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "ONNX_DETECTION");

    // 设置会话选项
    sessionOptions = Ort::SessionOptions();

    // 获取可用的执行提供者，检查是否有支持 GPU 的 CUDAExecutionProvider
    std::vector<std::string> availableProviders = Ort::GetAvailableProviders();
    auto cudaAvailable = std::find(availableProviders.begin(), availableProviders.end(), "CUDAExecutionProvider");
    OrtCUDAProviderOptions cudaOption;

    // 如果 GPU 可用并且用户要求使用 GPU，则配置为 GPU，否则使用 CPU
    if (isGPU && (cudaAvailable == availableProviders.end()))
    {
        std::cout << "GPU is not supported by your ONNXRuntime build. Fallback to CPU." << std::endl;
        std::cout << "Inference device: CPU" << std::endl;
    }
    else if (isGPU && (cudaAvailable != availableProviders.end()))
    {
        std::cout << "Inference device: GPU" << std::endl;
        sessionOptions.AppendExecutionProvider_CUDA(cudaOption); // 使用 GPU 执行
    }
    else
    {
        std::cout << "Inference device: CPU" << std::endl; // 使用 CPU 执行
    }

    // 加载 ONNX 模型文件，平台特定路径处理（Windows 和非 Windows）
#ifdef _WIN32
    std::wstring w_modelPath = utils::charToWstring(modelPath.c_str());
    session = Ort::Session(env, w_modelPath.c_str(), sessionOptions); // Windows 处理
#else
    session = Ort::Session(env, modelPath.c_str(), sessionOptions); // Linux 或其他系统处理
#endif

    Ort::AllocatorWithDefaultOptions allocator;

    // 获取模型的输入张量类型和形状信息
    Ort::TypeInfo inputTypeInfo = session.GetInputTypeInfo(0);
    std::vector<int64_t> inputTensorShape = inputTypeInfo.GetTensorTypeAndShapeInfo().GetShape();
    this->isDynamicInputShape = false;

    // 检查输入张量的宽高是否为动态（-1 表示动态大小）
    if (inputTensorShape[2] == -1 && inputTensorShape[3] == -1)
    {
        std::cout << "Dynamic input shape" << std::endl;
        this->isDynamicInputShape = true;
    }

    // 打印输入张量的形状
    for (auto shape : inputTensorShape)
        std::cout << "Input shape: " << shape << std::endl;

    // 获取输入和输出节点的名称
    auto input_name = session.GetInputNameAllocated(0, allocator);
    inputNodeNameAllocatedStrings.push_back(std::move(input_name));
    inputNames.push_back(inputNodeNameAllocatedStrings.back().get());

    auto output_name = session.GetOutputNameAllocated(0, allocator);
    outputNodeNameAllocatedStrings.push_back(std::move(output_name));
    outputNames.push_back(outputNodeNameAllocatedStrings.back().get());

    std::cout << "Input name: " << inputNames[0] << std::endl;
    std::cout << "Output name: " << outputNames[0] << std::endl;

    // 设置输入图像的大小
    this->inputImageShape = cv::Size2f(inputSize);
}

// 获取输出中最佳的类别信息，更新最佳类别ID和置信度
void YOLODetector::getBestClassInfo(std::vector<float>::iterator it, const int& numClasses,
                                    float& bestConf, int& bestClassId)
{
    // 第一到第五个元素是边界框信息和物体置信度
    bestClassId = 5;
    bestConf = 0;

    // 从第五个元素开始，遍历每个类别的置信度
    for (int i = 5; i < numClasses + 5; i++)
    {
        if (it[i] > bestConf)  // 找到置信度最大的类别
        {
            bestConf = it[i];
            bestClassId = i - 5;  // 更新类别ID
        }
    }
}

// 预处理输入图像，调整大小并转为浮动格式
void YOLODetector::preprocessing(cv::Mat &image, float*& blob, std::vector<int64_t>& inputTensorShape)
{
    cv::Mat resizedImage, floatImage;

    // 将图像从 BGR 转为 RGB
    cv::cvtColor(image, resizedImage, cv::COLOR_BGR2RGB);

    // 调整图像大小为指定的输入图像形状，并填充边缘
    utils::letterbox(resizedImage, resizedImage, this->inputImageShape,
                     cv::Scalar(114, 114, 114), this->isDynamicInputShape,
                     false, true, 32);

    // 更新输入张量的形状
    inputTensorShape[2] = resizedImage.rows;
    inputTensorShape[3] = resizedImage.cols;

    // 将图像转换为浮动格式并归一化到 0-1 范围
    resizedImage.convertTo(floatImage, CV_32FC3, 1 / 255.0);
    blob = new float[floatImage.cols * floatImage.rows * floatImage.channels()];
    cv::Size floatImageSize {floatImage.cols, floatImage.rows};

    // 从 HWC 转为 CHW 格式，以便用于 ONNX 输入
    std::vector<cv::Mat> chw(floatImage.channels());
    for (int i = 0; i < floatImage.channels(); ++i)
    {
        chw[i] = cv::Mat(floatImageSize, CV_32FC1, blob + i * floatImageSize.width * floatImageSize.height);
    }
    cv::split(floatImage, chw);  // 将图像分成不同的通道
}

// 后处理 ONNX 模型的输出，过滤检测框，进行 NMS
std::vector<Detection> YOLODetector::postprocessing(const cv::Size& resizedImageShape,
                                                    const cv::Size& originalImageShape,
                                                    std::vector<Ort::Value>& outputTensors,
                                                    const float& confThreshold, const float& iouThreshold)
{
    std::vector<cv::Rect> boxes;
    std::vector<float> confs;
    std::vector<int> classIds;

    // 获取输出张量数据
    auto* rawOutput = outputTensors[0].GetTensorData<float>();
    std::vector<int64_t> outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
    size_t count = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
    std::vector<float> output(rawOutput, rawOutput + count);

    // 计算类别数
    int numClasses = (int)outputShape[2] - 5;
    int elementsInBatch = (int)(outputShape[1] * outputShape[2]);

    // 对每个检测结果进行处理
    for (auto it = output.begin(); it != output.begin() + elementsInBatch; it += outputShape[2])
    {
        float clsConf = it[4];

        if (clsConf > confThreshold)  // 如果置信度超过阈值
        {
            int centerX = (int) (it[0]);
            int centerY = (int) (it[1]);
            int width = (int) (it[2]);
            int height = (int) (it[3]);
            int left = centerX - width / 2;
            int top = centerY - height / 2;

            // 获取类别置信度和类别ID
            float objConf;
            int classId;
            this->getBestClassInfo(it, numClasses, objConf, classId);

            float confidence = clsConf * objConf;

            // 存储检测框，置信度和类别ID
            boxes.emplace_back(left, top, width, height);
            confs.emplace_back(confidence);
            classIds.emplace_back(classId);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confs, confThreshold, iouThreshold, indices);  // 使用 NMS 过滤重叠框

    std::vector<Detection> detections;

    // 将检测框坐标缩放回原图像尺寸
    for (int idx : indices)
    {
        Detection det;
        det.box = cv::Rect(boxes[idx]);
        utils::scaleCoords(resizedImageShape, det.box, originalImageShape);

        det.conf = confs[idx];
        det.classId = classIds[idx];
        detections.emplace_back(det);  // 存储检测结果
    }

    return detections;
}

// 执行目标检测，返回检测结果
std::vector<Detection> YOLODetector::detect(cv::Mat &image, const float& confThreshold = 0.4,
                                            const float& iouThreshold = 0.45)
{
    float *blob = nullptr;
    std::vector<int64_t> inputTensorShape {1, 3, -1, -1};  // 输入张量形状
    this->preprocessing(image, blob, inputTensorShape);  // 图像预处理

    size_t inputTensorSize = utils::vectorProduct(inputTensorShape);

    std::vector<float> inputTensorValues(blob, blob + inputTensorSize);  // 构建输入张量的值

    std::vector<Ort::Value> inputTensors;

    // 创建 CPU 内存信息
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
            OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    inputTensors.push_back(Ort::Value::CreateTensor<float>(
            memoryInfo, inputTensorValues.data(), inputTensorSize,
            inputTensorShape.data(), inputTensorShape.size()
    ));

    // 执行模型推理
    std::vector<Ort::Value> outputTensors = this->session.Run(Ort::RunOptions{nullptr},
                                                              inputNames.data(),
                                                              inputTensors.data(),
                                                              1,
                                                              outputNames.data(),
                                                              1);

    // 后处理输出并获取最终的检测结果
    cv::Size resizedShape = cv::Size((int)inputTensorShape[3], (int)inputTensorShape[2]);
    std::vector<Detection> result = this->postprocessing(resizedShape,
                                                         image.size(),
                                                         outputTensors,
                                                         confThreshold, iouThreshold);

    delete[] blob;  // 释放预处理分配的内存

    return result;  // 返回检测结果
}

