# Realsense

## Tips
1. `rs2::depth_frame.get_data()` 获得的即为真实的距离。返回数据格式为`uint16_t`，单位为毫米，整数运算快。
2. 归一化为[0.0f, 1.0f]时，最终要转换为`float`类型，接下来的归一化操作巨耗时，会将FPS从90hz拉低到40hz！