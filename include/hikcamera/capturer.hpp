#pragma once
#include <chrono>
#include <expected>
#include <opencv2/core/mat.hpp>

namespace hikcamera {
using Milli = std::chrono::duration<float, std::milli>;

struct Profile final {
    float exposure_ms = 2.;
    float gain = 16.9807; // Max for our camera
    float frame_rate = 80.;

    Milli timeout{1.0};

    bool trigger_mode = false;
    bool invert_image = false;
    bool software_sync = false;
};

class Camera final {
public:
    auto initialize(const Profile& profile = {}) noexcept
        -> std::expected<std::string, std::string>;

    auto reset_connection() noexcept -> void;

    auto get_size() const noexcept -> std::expected<cv::Size2i, std::string_view>;

    auto read_image_with_expected() noexcept -> std::expected<cv::Mat, std::string>;

    /// @throw std::runtime_error
    auto read_image_with_exception() -> cv::Mat;

public:
    explicit Camera() noexcept;
    ~Camera() noexcept;

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace hikcamera
