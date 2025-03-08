#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <chrono>


int main() {
    // 参数设置
    int saved_count = 0;
    int width = 640;
    int height = 480;
    int fps = 30;
    int ALIGN_WAY = 1; // 0: 彩色图像对齐到深度图; 1: 深度图对齐到彩色图像

    // 创建保存路径
    std::string color_path, depth_path;

    // 创建管道和配置
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);

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

            // 如果需要彩色图像对齐到深度图，获取未对齐的彩色图像
            cv::Mat color_image2;
            if (ALIGN_WAY == 0) {
                rs2::video_frame color_frame2 = frames.get_color_frame();
                color_image2 = cv::Mat(cv::Size(width, height), CV_8UC3, (void*)color_frame2.get_data(), cv::Mat::AUTO_STEP);
                cv::imshow("color_image2", color_image2);
            }

            // 将深度图转换为米为单位
            cv::Mat depth_image_in_meters;
            depth_image.convertTo(depth_image_in_meters, CV_32F, depth_scale);

            // 归一化深度图到 0-255 范围（灰度显示）
            cv::Mat depth_normalized;
            cv::normalize(depth_image_in_meters, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

            // 显示彩色图和归一化的深度图
            cv::imshow("Color Image", color_image);
            cv::imshow("Depth Image (Normalized)", depth_normalized);

            // 按键处理
            char key = cv::waitKey(1);
            if (key == 's') {
                saved_count++;
                std::cout << saved_count << " 已保存图像至 " << color_path << " 和 " << depth_path << std::endl;

                // 保存彩色图像
                if (color_image2.empty()) {
                    cv::imwrite(color_path + "/" + std::to_string(saved_count) + ".png", color_image);
                } else {
                    cv::imwrite(color_path + "/" + std::to_string(saved_count) + ".png", color_image2);
                }

                // 保存深度图（以 .npy 格式保存）
                std::string depth_file = depth_path + "/" + std::to_string(saved_count) + ".npy";
                cv::FileStorage fs(depth_file, cv::FileStorage::WRITE);
                fs << "depth" << depth_image_in_meters;
                fs.release();
            } else if (key == 'q' || key == 27) {
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