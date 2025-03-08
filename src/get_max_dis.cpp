#include <librealsense2/rs.hpp> // 包含 RealSense 头文件
#include <opencv2/opencv.hpp>   // 包含 OpenCV 头文件
#include <iostream>
#include <algorithm>

int main() {
    // 创建 RealSense 管道
    rs2::pipeline pipe;
    rs2::config cfg;

    // 启用深度流和彩色流
    /* 424*240, 480*270, 640*360, 640*480, 848*480, 1280*720
       6Hz, 15Hz, 30Hz, 60Hz, 90Hz
    */
    cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 1280, 720, rs2_format::RS2_FORMAT_Z16, 30);
    cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 1280, 720, rs2_format::RS2_FORMAT_BGR8, 30);

    // 启动管道
    rs2::pipeline_profile profile = pipe.start(cfg);

    try {
        while (true) {
            // 等待下一组帧（深度帧和彩色帧）
            rs2::frameset frames = pipe.wait_for_frames();

            // 获取深度帧和彩色帧
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            rs2::video_frame color_frame = frames.get_color_frame();

            // 获取深度图的宽度和高度
            int width = depth_frame.get_width();
            int height = depth_frame.get_height();

            // 获取深度数据
            const uint16_t* depth_data = (const uint16_t*)depth_frame.get_data();

            // 初始化最大距离及其坐标
            float max_distance = 0.0f;
            int max_x = 0, max_y = 0;

            // 遍历深度图，找到最大距离及其坐标
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    // 获取当前像素的深度值（单位为毫米）
                    uint16_t depth_value = depth_data[y * width + x];

                    // 将深度值转换为米
                    float distance = depth_value * depth_frame.get_units();

                    // 更新最大距离及其坐标
                    if (distance > max_distance) {
                        max_distance = distance;
                        max_x = x;
                        max_y = y;
                    }
                }
            }

            // 输出最大距离
            std::cout << "Max distance in the depth frame: " << max_distance << " meters" << std::endl;

            // 将深度帧转换为 OpenCV 格式
            cv::Mat depth_image(cv::Size(width, height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_image_8u;
            depth_image.convertTo(depth_image_8u, CV_8U, 255.0 / 5000); // 将深度图归一化到 0-255

            // 将彩色帧转换为 OpenCV 格式
            cv::Mat color_image(cv::Size(width, height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

            // 在深度图中标记最远的点
            cv::circle(depth_image_8u, cv::Point(max_x, max_y), 10, cv::Scalar(0, 0, 255), 2); // 红色圆圈

            // 在彩色图中标记最远的点
            cv::circle(color_image, cv::Point(max_x, max_y), 10, cv::Scalar(0, 0, 255), 2); // 红色圆圈

            // 显示深度图和彩色图
            cv::imshow("Depth Image", depth_image_8u);
            cv::imshow("Color Image", color_image);

            // 按下 ESC 键退出
            if (cv::waitKey(1) == 27) {
                break;
            }
        }
    }
    catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // 停止管道
    pipe.stop();

    return EXIT_SUCCESS;
}