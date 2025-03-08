#include <librealsense2/rs.hpp> // 包含 RealSense 跨平台 API
#include <opencv2/opencv.hpp>   // 包含 OpenCV API
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

int main() {
    // 声明 RealSense 管道，封装实际设备和传感器
    rs2::pipeline pipe;

    // 创建配置以使用非默认配置配置管道
    rs2::config cfg;

    // 添加一个流到配置中
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 90);

    // 使用选择的配置启动管道
    rs2::pipeline_profile profile = pipe.start(cfg);

    // 定义窗口名称以显示深度图像
    const char* depth_window = "Depth Image";

    // 创建 OpenCV 窗口以显示深度图像
    namedWindow(depth_window, WINDOW_AUTOSIZE);

    // 用于计算帧率的变量和时钟
    int frames_count = 0;
    auto start = chrono::steady_clock::now();

    // 循环直到有人关闭窗口
    while (waitKey(1) < 0 && getWindowProperty(depth_window, WND_PROP_AUTOSIZE) >= 0) {
        // 等待从相机获取下一组帧
        auto frames = pipe.wait_for_frames();

        // 从管道获取帧
        rs2::frame depth_frame = frames.get_depth_frame();

        // 查询帧的大小（宽度和高度）
        int width = depth_frame.as<rs2::video_frame>().get_width();
        int height = depth_frame.as<rs2::video_frame>().get_height();

        // 从深度帧数据创建 OpenCV 矩阵（大小为 height x width）
        Mat depth_image(Size(width, height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

        // CLIP
        // 将深度图裁剪到0m到6m的范围（深度单位是毫米）
        for (int i = 0; i < depth_image.rows; ++i) {
            for (int j = 0; j < depth_image.cols; ++j) {
                uint16_t depth_value = depth_image.at<uint16_t>(i, j);
                // 转换为米并裁剪
                if (depth_value < 100) {
                    depth_image.at<uint16_t>(i, j) = 0;
                }
                else if (depth_value > 6000) {
                    depth_image.at<uint16_t>(i, j) = 6000;
                }
            }
        }

        // 归一化深度图像以便正确显示
        Mat depth_normalized;
        normalize(depth_image, depth_normalized, 0, 255, NORM_MINMAX, CV_8U);

        // 应用伪彩色映射到深度图像
        Mat depth_colormap;
        applyColorMap(depth_normalized, depth_colormap, COLORMAP_JET);

        // 计算帧率
        frames_count++; 
        auto now = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - start).count() / 1000.0;
        double fps = frames_count / elapsed;

        // 在深度图像窗口上方显示帧率
        putText(depth_colormap, "FPS: " + to_string(fps), Point(10, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 2);

        // 更新窗口显示新数据
        imshow(depth_window, depth_colormap);
    }

    // 停止管道并释放资源
    pipe.stop();

    return 0;
}
