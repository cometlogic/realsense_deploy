#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 创建 RealSense 管道
    rs2::pipeline p;
    rs2::config cfg;

    // 配置深度流，640x480分辨率，30帧率
    /* 424*240, 480*270, 640*360, 640*480, 848*480, 1280*720
       6Hz, 15Hz, 30Hz, 60Hz, 90Hz
    */
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 90);

    // 配置并启动管道
    p.start(cfg);

    // 设置裁剪距离范围（单位：米）
    float min_distance = 0.1f; // 最小距离
    float max_distance = 5.0f; // 最大距离

    // 创建滤波器
    rs2::decimation_filter decimation_filter;
    rs2::hole_filling_filter hole_filling;
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;

    // 滤波器参数设置
    decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2); // 降采样滤波器，降低分辨率
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 5); // 空间滤波器，平滑深度图像
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5); // 平滑系数
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 50); // 平滑阈值
    spatial_filter.set_option(RS2_OPTION_HOLES_FILL, 5); // 填充孔洞
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temporal_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
    temporal_filter.set_option(RS2_OPTION_HOLES_FILL, 3);

    // 创建 OpenCV 窗口
    cv::namedWindow("Depth Image", cv::WINDOW_NORMAL);

    // 计时器变量，用于计算帧率
    auto last_time = std::chrono::high_resolution_clock::now();

    while (true) {
        // 等待帧数据到达
        rs2::frameset frames = p.wait_for_frames();

        // 获取深度图
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        rs2::depth_frame filtered = depth_frame;

        filtered = spatial_filter.process(filtered);
        filtered = temporal_filter.process(filtered);
        filtered = hole_filling.process(filtered);
        filtered = decimation_filter.process(filtered);

        // 获取深度图的宽度和高度
        int width = filtered.get_width();
        int height = filtered.get_height();

        // 创建 OpenCV Mat 来存储深度图
        cv::Mat depth_image = cv::Mat::ones(height, width, CV_32F);
        // cv::Mat depth_image_(cv::Size(width, height), CV_16UC1, (void*)filtered.get_data(), cv::Mat::AUTO_STEP);

        // 裁剪深度图的像素值到设定的距离范围
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // 获取当前像素的深度值（单位：米）
                float depth_value = filtered.get_distance(x, y);
                // uint16_t depth_value_mm = depth_image_.at<uint16_t>(y, x);
                // std::cout << "depth_value:" << depth_value << " depth_value_mm:" << depth_value_mm << std::endl;
                depth_image.at<float>(y, x) = depth_value / max_distance;
                if (std::isnan(depth_value)){
                    depth_image.at<float>(y, x) = max_distance / max_distance;
                }
                if (depth_value < min_distance) {
                    depth_image.at<float>(y, x) = 0.0f / max_distance;
                }
                else if (depth_value > max_distance) {
                    depth_image.at<float>(y, x) = max_distance / max_distance;
                }
            }
        }

        // for (int i = 0; i < depth_image.rows; ++i) {
        //     for (int j = 0; j < depth_image.cols; ++j) {
        //         std::cout << depth_image.at<float>(i, j) << " ";
        //     }
        //     std::cout << std::endl;
        // }

        // 将浮点图像转换为 8 位图像
        cv::Mat final_depth_image;
        depth_image.convertTo(final_depth_image, CV_8U, 255.0);

        // 计算和显示帧率
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = current_time - last_time;
        double fps = 1.0 / duration.count();
        last_time = current_time;

        std::string fps_text = "FPS: " + std::to_string(fps);

        // 获取当前分辨率
        cv::Size depth_size = final_depth_image.size();

        cv::putText(final_depth_image, fps_text, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

        // 显示当前分辨率
        cv::putText(final_depth_image, "Resolution: " + std::to_string(depth_size.width) + "x" + std::to_string(depth_size.height), cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

        // 显示裁剪后的深度图
        cv::imshow("Depth Image", final_depth_image);

        // 按下 ESC 键退出
        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    return 0;
}