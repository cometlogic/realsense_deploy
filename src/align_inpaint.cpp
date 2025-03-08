#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

// 修补深度图像
cv::Mat inpaint_depth_image(const cv::Mat& depth_image, int inpaint_radius = 3) {
    // 创建掩码，标记无效像素（值为 0）
    cv::Mat mask = (depth_image == 0);
    mask.convertTo(mask, CV_8U); // 转换为 8 位单通道图像

    // 将无效像素设置为 NaN
    cv::Mat depth_image_fixed;
    depth_image.convertTo(depth_image_fixed, CV_32F);
    depth_image_fixed.setTo(std::numeric_limits<float>::quiet_NaN(), mask);

    // 使用线性插值填充 NaN 值
    cv::Mat valid_mask = ~mask;
    cv::Mat inpainted_depth_image;
    cv::inpaint(depth_image_fixed, mask, inpainted_depth_image, inpaint_radius, cv::INPAINT_TELEA);

    return inpainted_depth_image;
}

int main() {
    // 参数设置
    int width = 640;
    int height = 480;
    int fps = 30;
    int ALIGN_WAY = 1; // 0: 彩色图像对齐到深度图; 1: 深度图对齐到彩色图像

    // 创建管道和配置
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 90);
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);

    // 启动管道
    rs2::pipeline_profile profile = pipeline.start(config);

    // 获取深度比例
    float depth_scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();

    // 设置对齐方式
    rs2::align align(ALIGN_WAY == 1 ? RS2_STREAM_COLOR : RS2_STREAM_DEPTH);

    try {
        while (true) {
            // 等待帧数据
            rs2::frameset frames = pipeline.wait_for_frames();

            // 对齐帧
            rs2::frameset aligned_frames = align.process(frames);

            // 获取深度帧和彩色帧
            rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
            rs2::video_frame color_frame = aligned_frames.get_color_frame();

            // 将深度帧和彩色帧转换为 OpenCV Mat
            cv::Mat depth_image(cv::Size(width, height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat color_image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

            // 将深度图转换为米为单位
            cv::Mat depth_image_in_meters;
            depth_image.convertTo(depth_image_in_meters, CV_32F, depth_scale);

            // 修补深度图像
            cv::Mat inpainted_depth_image = inpaint_depth_image(depth_image_in_meters);

            // 处理无效值
            inpainted_depth_image.setTo(1.0f, inpainted_depth_image <= 0.5f);

            // 中值滤波
            cv::Mat median_filtered_image;
            cv::medianBlur(inpainted_depth_image, median_filtered_image, 5);

            // 双边滤波
            cv::Mat filtered_image;
            cv::bilateralFilter(median_filtered_image, filtered_image, 5, 75, 75);

            // 显示彩色图和处理后的深度图
            cv::imshow("Color Image", color_image);
            cv::imshow("Filtered Depth Image", filtered_image);

            // 按键处理
            char key = cv::waitKey(1);
            if (key == 'q' || key == 27) {
                break;
            }
        }
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // 停止管道
    pipeline.stop();
    cv::destroyAllWindows();

    return 0;
}