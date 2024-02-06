/**
 * @author Qzh (zihanqin2048@gmail.com)
 * @brief Hikcamera
 * @copyright Copyright (c) 2024 by Alliance, All Rights Reserved.
 */
#pragma once

#include <chrono>
#include <ratio>

#include <opencv2/core/mat.hpp>

namespace hikcamera {

class ImageCapturer final {
public:
    struct CameraProfile {
        CameraProfile() noexcept {
            using namespace std::chrono_literals;
            trigger_mode  = false;
            invert_image  = false;
            exposure_time = 5ms;
            gain          = 0;
        }

        bool trigger_mode;
        bool invert_image;

        std::chrono::duration<float, std::micro> exposure_time;
        float gain;
    };

    ImageCapturer(
        const CameraProfile& profile = CameraProfile{}, const char* user_defined_name = nullptr);

    ImageCapturer(const ImageCapturer&)            = delete;
    ImageCapturer& operator=(const ImageCapturer&) = delete;

    ~ImageCapturer();

    [[nodiscard]] cv::Mat
        read(std::chrono::duration<unsigned int, std::micro> timeout = std::chrono::seconds(5));

private:
    class Impl;
    Impl* impl_;
};

} // namespace hikcamera