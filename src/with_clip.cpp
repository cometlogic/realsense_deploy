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

    float depth_clipping_distance = 5.0f; // 深度裁剪距离

    // 启动管道
    p.start(cfg);

    // 创建滤波器
    rs2::decimation_filter decimation_filter;
    rs2::hole_filling_filter hole_filling;
    rs2::spatial_filter spatial_filter;
    rs2::temporal_filter temporal_filter;

    // 计时器变量，用于计算帧率
    auto last_time = std::chrono::high_resolution_clock::now();

    while (true) {
        // 获取一帧数据
        rs2::frameset frames = p.wait_for_frames();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        // 获取深度数据
        const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth_frame.get_data());

        // 将深度数据转换为OpenCV Mat
        cv::Mat depth_image(depth_frame.get_height(), depth_frame.get_width(), CV_16U, const_cast<uint16_t*>(depth_data));

        // 应用Hole-filling滤波器
        rs2::frame hole_filled_frame = hole_filling.process(depth_frame);
        const uint16_t* hole_filled_data = reinterpret_cast<const uint16_t*>(hole_filled_frame.get_data());
        cv::Mat hole_filled_image(depth_frame.get_height(), depth_frame.get_width(), CV_16U, const_cast<uint16_t*>(hole_filled_data));

        // 应用Spatial filter
        rs2::frame spatial_filtered_frame = spatial_filter.process(hole_filled_frame);
        const uint16_t* spatial_filtered_data = reinterpret_cast<const uint16_t*>(spatial_filtered_frame.get_data());
        cv::Mat spatial_filtered_image(depth_frame.get_height(), depth_frame.get_width(), CV_16U, const_cast<uint16_t*>(spatial_filtered_data));

        // 应用Temporal filter
        rs2::frame temporal_filtered_frame = temporal_filter.process(spatial_filtered_frame);
        const uint16_t* temporal_filtered_data = reinterpret_cast<const uint16_t*>(temporal_filtered_frame.get_data());
        cv::Mat temporal_filtered_image(depth_frame.get_height(), depth_frame.get_width(), CV_16U, const_cast<uint16_t*>(temporal_filtered_data));

        // 将深度图裁剪到0m到6m的范围（深度单位是毫米）
        for (int i = 0; i < temporal_filtered_image.rows; ++i) {
            for (int j = 0; j < temporal_filtered_image.cols; ++j) {
                uint16_t depth_value = temporal_filtered_image.at<uint16_t>(i, j);
                // 转换为米并裁剪
                float depth_in_meters = depth_value * 0.001f; // 毫米转换为米
                if (depth_in_meters > depth_clipping_distance) {
                    temporal_filtered_image.at<uint16_t>(i, j) = depth_clipping_distance; // 超过6米的深度值设为0
                } else if (depth_in_meters < 0.0f) {
                    temporal_filtered_image.at<uint16_t>(i, j) = 0; // 小于0的深度值设为0
                }
            }
        }

        // 将裁剪后的深度图转换为灰度图，重新映射到0-255范围
        cv::Mat final_depth_image;
        temporal_filtered_image.convertTo(final_depth_image, CV_8U, 0.1);  // 重新转换为灰度图

        // 计算和显示帧率
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = current_time - last_time;
        double fps = 1.0 / duration.count();
        last_time = current_time;

        // 构建帧率文本
        std::string fps_text = "FPS: " + std::to_string(fps);

        // 在图像的左上角添加帧率文本
        cv::putText(final_depth_image, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);

        // 显示最终深度图
        cv::imshow("Processed Depth Image", final_depth_image);

        // 按 'q' 键退出
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
