# Hikcamera

`海康相机 MV-CS016-10UC` 的 C++ 封装，使用 `Ament Cmake` 管理

## 项目构建

项目依赖 `ROS2` 环境，需要使用团队提供的 `Docker` 镜像开发，此处我们默认读者已经了解过 `ROS2` 及 `RMCS` 相关知识和操作

```sh
# 进入容器中
cd /path/to/rmcs_ws/
git clone https://github.com/Alliance-Algorithm/ros2-hikcamera.git src/hikcamera

# 使用 ROS2 工具构建
colcon build --packages-select hikcamera --symlink-install --merge-install

# 或者使用 RMCS 脚本
build-rmcs --packages-select hikcamera
```

## 项目引入

在 `CMake` 中加入下面的配置即可，注意要先 `source` 上面步骤构建好的 `install/setup.zsh`，才能正常构建：

```cmake
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(hikcamera REQUIRED)

include_directories(${hikcamera_INCLUDE_DIRS})

target_link_libraries(
    your_app
    ${OpenCV_LIBS}
    ${hikcamera_LIBRARIES}
)
```

可以参考：[`rmcs_auto_aim_v2/test/CMakeLists.txt`](https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2/blob/main/test/CMakeLists.txt)

## 基本使用

接口使用 `C++23` 所引入的 `expected`，能够携带更加自由的错误上下文，并强制约束调用者检查，是出于工程化的考量

```cpp
// 1. 配置
auto config = hikcamera::Config {
    .timeout_ms  = 2'000,
    .exposure_us = 1'500,
    // ...
};
auto camera = hikcamera::Camera {};
camera.configure(config);

// 2. 连接相机
//  - result: std::expected<void, std::string>
if (auto result = camera.connect()) {
    std::println("Camera connect successfully");
} else {
    std::println("Failed to connect: {}", result.error());
}

// 3. 读取图片
//  - mat: std::expected<cv::Mat, std::string>
//  或者使用 ‘read_image_with_timestamp’
//  调用该接口会阻塞等待，需要特别注意
if (auto mat = camera.read_image()) {
    std::print("Read a image as cv::Mat");
} else {
    std::print("Failed to read: {}", mat.error());
}

// 4. 断开相机
//  忽略返回值，有时候断开操作也会报错，但我们不需要处理
if(camera.connected())
    std::ignore = camera.disconnect();
```

## 相关示例

- 循环捕获：[`rmcs_auto_aim_v2/test/hikcamera.cpp`](https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2/blob/main/test/hikcamera.cpp)

- 视频推流：[`rmcs_auto_aim_v2/test/streaming.cpp`](https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2/blob/main/test/streaming.cpp)

## 故障排除

- **未找到相机但是使用 `lsusb` 能找到**
    
    一般是用户没有权限导致的，我们需要在使用摄像头的电脑上配置 udev 规则，注意不是在 `Docker` 容器中：
    ```sh
    # 创建 Rules 文件
    echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2bdf\", ATTR{idProduct}==\"0001\", MODE=\"0666\"" | sudo tee /etc/udev/rules.d/99-hikcamera.rules
    # 重新加载 Rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
    执行完上述指令后，理论上就能正确查找相机了