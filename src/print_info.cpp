#include <librealsense2/rs.hpp>
#include <iostream>

int main(){

    // 获取设备列表
    rs2::context ctx;
    auto devices = ctx.query_devices();

    // 检查每个设备的流配置
    for (auto&& dev : devices) {
        std::cout << "Device: " << dev.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        for (auto&& stream : dev.get_stream_profiles()) {
            std::cout << "Stream: " 
                    << stream.stream_name() 
                    << ", Resolution: " 
                    << stream.as<rs2::video_stream_profile>().width() 
                    << "x" 
                    << stream.as<rs2::video_stream_profile>().height() 
                    << ", FPS: " 
                    << stream.as<rs2::video_stream_profile>().fps() 
                    << std::endl;
        }
    }
}