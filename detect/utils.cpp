#include "detect/utils.h"


// 计算向量中所有元素的乘积
size_t utils::vectorProduct(const std::vector<int64_t>& vector)
{
    if (vector.empty())  // 如果向量为空，返回0
        return 0;

    size_t product = 1;  // 初始化乘积为1
    for (const auto& element : vector)  // 遍历向量中的每个元素
        product *= element;  // 计算元素的乘积

    return product;  // 返回乘积
}

// 将C字符串转换为wstring类型
std::wstring utils::charToWstring(const char* str)
{
    typedef std::codecvt_utf8<wchar_t> convert_type;  // 使用UTF-8转换类型
    std::wstring_convert<convert_type, wchar_t> converter;  // 创建转换器

    return converter.from_bytes(str);  // 将字节字符串转换为wstring
}

// 从文件中加载类别名称
std::vector<std::string> utils::loadNames(const std::string& path)
{
    std::vector<std::string> classNames;  // 用于存储类别名称的向量
    std::ifstream infile(path);  // 打开文件

    if (infile.good())  // 如果文件打开成功
    {
        std::string line;
        while (getline(infile, line))  // 逐行读取文件
        {
            if (line.back() == '\r')  // 去除Windows下的回车符
                line.pop_back();
            classNames.emplace_back(line);  // 将读取的行添加到classNames向量
        }
        infile.close();  // 关闭文件
    }
    else
    {
        std::cerr << "ERROR: Failed to access class name path: " << path << std::endl;
    }

    return classNames;  // 返回类别名称的向量
}

// 可视化检测结果，将检测框和标签绘制在图像上
void utils::visualizeDetection(cv::Mat& image, std::vector<Detection>& detections,
                               const std::vector<std::string>& classNames)
{
    for (const Detection& detection : detections)  // 遍历所有检测结果
    {
        // 绘制矩形框
        cv::rectangle(image, detection.box, cv::Scalar(229, 160, 21), 2);

        int x = detection.box.x;
        int y = detection.box.y;

        // 计算置信度百分比并构建标签
        int conf = (int)std::round(detection.conf * 100);
        int classId = detection.classId;
        std::string label = classNames[classId] + " 0." + std::to_string(conf);

        // 计算文本框的大小和基准线
        int baseline = 0;
        cv::Size size = cv::getTextSize(label, cv::FONT_ITALIC, 0.8, 2, &baseline);
        
        // 绘制背景矩形框
        cv::rectangle(image,
                      cv::Point(x, y - 25), cv::Point(x + size.width, y),
                      cv::Scalar(229, 160, 21), -1);

        // 绘制标签文本
        cv::putText(image, label,
                    cv::Point(x, y - 3), cv::FONT_ITALIC,
                    0.8, cv::Scalar(255, 255, 255), 2);
    }
}

// 调整图像大小并填充，以适应新的输入形状，保持比例或填充背景
void utils::letterbox(const cv::Mat& image, cv::Mat& outImage,
                      const cv::Size& newShape = cv::Size(640, 640),
                      const cv::Scalar& color = cv::Scalar(114, 114, 114),
                      bool auto_ = true,
                      bool scaleFill = false,
                      bool scaleUp = true,
                      int stride = 32)
{
    cv::Size shape = image.size();
    // 计算图像的缩放比例
    float r = std::min((float)newShape.height / (float)shape.height,
                       (float)newShape.width / (float)shape.width);
    if (!scaleUp)
        r = std::min(r, 1.0f);  // 不允许放大图片

    float ratio[2] {r, r};  // 保持宽高比例
    int newUnpad[2] {(int)std::round((float)shape.width * r),
                     (int)std::round((float)shape.height * r)};

    // 计算需要填充的宽度和高度
    auto dw = (float)(newShape.width - newUnpad[0]);
    auto dh = (float)(newShape.height - newUnpad[1]);

    // 如果auto_为true，保证宽高可以被stride整除
    if (auto_)
    {
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    }
    else if (scaleFill)
    {
        dw = 0.0f;
        dh = 0.0f;
        newUnpad[0] = newShape.width;
        newUnpad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }

    dw /= 2.0f;  // 平均分配填充
    dh /= 2.0f;

    // 调整图像大小
    if (shape.width != newUnpad[0] && shape.height != newUnpad[1])
    {
        cv::resize(image, outImage, cv::Size(newUnpad[0], newUnpad[1]));
    }

    // 进行边框填充
    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

// 根据新图像形状调整坐标，以适应原始图像尺寸
void utils::scaleCoords(const cv::Size& imageShape, cv::Rect& coords, const cv::Size& imageOriginalShape)
{
    float gain = std::min((float)imageShape.height / (float)imageOriginalShape.height,
                          (float)imageShape.width / (float)imageOriginalShape.width);

    int pad[2] = {(int) (( (float)imageShape.width - (float)imageOriginalShape.width * gain) / 2.0f),
                  (int) (( (float)imageShape.height - (float)imageOriginalShape.height * gain) / 2.0f)};

    // 计算新的坐标位置
    coords.x = (int) std::round(((float)(coords.x - pad[0]) / gain));
    coords.y = (int) std::round(((float)(coords.y - pad[1]) / gain));

    coords.width = (int) std::round(((float)coords.width / gain));
    coords.height = (int) std::round(((float)coords.height / gain));

    // 这里是对坐标进行裁剪的地方，但被注释掉了
    // coords.x = utils::clip(coords.x, 0, imageOriginalShape.width);
    // coords.y = utils::clip(coords.y, 0, imageOriginalShape.height);
    // coords.width = utils::clip(coords.width, 0, imageOriginalShape.width);
    // coords.height = utils::clip(coords.height, 0, imageOriginalShape.height);
}

// 对值进行裁剪，使其在给定的范围内
template <typename T>
T utils::clip(const T& n, const T& lower, const T& upper)
{
    return std::max(lower, std::min(n, upper));  // 返回限制后的值
}
