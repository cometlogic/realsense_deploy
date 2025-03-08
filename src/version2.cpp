#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main() {
    // 创建管道对象
    rs2::pipeline p;
    rs2::config cfg;

    // 配置深度流，640x480分辨率，30帧率
    /* 424*240, 480*270, 640*360, 640*480, 848*480, 1280*720
       6Hz, 15Hz, 30Hz, 60Hz, 90Hz
    */
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 90);

    float depth_clipping_distance[2] = {0.1f, 5.0f}; // 深度裁剪距离

    // 启动管道
    p.start(cfg);

    // 创建滤波器
    rs2::decimation_filter decimation_filter;
    rs2::hole_filling_filter hole_filling;
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;

    // 滤波器参数设置
    decimation_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2); // 降采样滤波器，降低分辨率
    spatial_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 5); // 空间滤波器，平滑深度图像
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 1); // 平滑系数
    spatial_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 50); // 平滑阈值
    spatial_filter.set_option(RS2_OPTION_HOLES_FILL, 3); // 填充孔洞

    // 计时器变量，用于计算帧率
    auto last_time = std::chrono::high_resolution_clock::now();

    while (true) {
        // 获取一帧数据
        rs2::frameset frames = p.wait_for_frames();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        rs2::depth_frame filtered = depth_frame;

        filtered = decimation_filter.process(filtered);
        filtered = spatial_filter.process(filtered);
        filtered = temporal_filter.process(filtered);
        filtered = hole_filling.process(filtered);

        // 获取深度数据
        const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(filtered.get_data());

        // 将深度数据转换为OpenCV Mat
        cv::Mat depth_image(filtered.get_height(), filtered.get_width(), CV_16U, const_cast<uint16_t*>(depth_data));

        // 打印depth_image中的数据
        // for (int i = 0; i < depth_image.rows; ++i) {
        //     for (int j = 0; j < depth_image.cols; ++j) {
        //         std::cout << depth_image.at<uint16_t>(i, j) * depth_frame.get_units() << " ";
        //     }
        //     std::cout << std::endl;
        // }

        // 将深度图裁剪到0m到6m的范围（深度单位是毫米）
        for (int i = 0; i < depth_image.rows; ++i) {
            for (int j = 0; j < depth_image.cols; ++j) {
                uint16_t depth_value = depth_image.at<uint16_t>(i, j);
                // 转换为米并裁剪
                // float depth_in_meters = depth_value * 0.001f; // 毫米转换为米
                if (depth_value > depth_clipping_distance[1] / 0.001f) {
                    depth_image.at<uint16_t>(i, j) = depth_clipping_distance[1] / 0.001f; // 超过6米的深度值设为6
                } else if (depth_value < depth_clipping_distance[0] / 0.001f) {
                    depth_image.at<uint16_t>(i, j) = depth_clipping_distance[0] / 0.001f; // 小于0的深度值设为0
                }
            }
        }

        // 将裁剪后的深度图转换为灰度图，重新映射到0-255范围
        cv::Mat final_depth_image;
        depth_image.convertTo(final_depth_image, CV_8U, 0.1);  // 重新转换为灰度图

        // save depth_image to file
        // cv::imwrite("depth_image.jpg", final_depth_image);

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

        // 以1280*720分辨率显示
        cv::resize(final_depth_image, final_depth_image, cv::Size(1280, 720));

        cv::imshow("Depth Image", final_depth_image);

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
