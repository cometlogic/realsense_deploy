#include <opencv2/opencv.hpp>

int main() {
    // 创建一个浮点图像，像素值在 [0, 1] 范围内
    cv::Mat float_image(1000, 1000, CV_32F);
    cv::randu(float_image, 0.0, 1.0); // 随机生成 [0, 1] 范围内的值

    // 将浮点图像转换为 8 位图像
    cv::Mat uint8_image;
    float_image.convertTo(uint8_image, CV_8U, 255.0);

    // 显示图像
    cv::imshow("Float Image", float_image); // 可能会显示异常（全黑）
    cv::imshow("Uint8 Image", uint8_image); // 正常显示

    cv::waitKey(0);
    return 0;
}