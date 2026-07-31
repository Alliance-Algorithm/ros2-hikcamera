#pragma once
#include <chrono>
#include <expected>
#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

namespace hikcamera {
static constexpr auto kMaxGain = float{16.9807};

struct Config {
    unsigned int timeout_ms = 2000;

    std::string camera_name = {};

    float exposure_us = 2000.;
    float framerate = 80;
    float gain = kMaxGain;

    bool invert_image = false;
    bool software_sync = false;

    bool trigger_mode = false;
    bool fixed_framerate = true;
};

/// Enumerate currently visible Hik cameras and return each device's UserDefinedName.
/// Empty/null names are returned as empty strings.
auto list_camera_names() noexcept -> std::expected<std::vector<std::string>, std::string>;

class Camera {
public:
    struct Image {
        using Clock = std::chrono::steady_clock;
        using Stamp = Clock::time_point;

        cv::Mat mat;
        Stamp timestamp;
    };

    explicit Camera() noexcept;
    ~Camera() noexcept;

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    auto configure(const Config&) noexcept -> void;

    auto connect() noexcept -> std::expected<void, std::string>;

    auto disconnect() noexcept -> std::expected<void, std::string>;

    auto connected() const noexcept -> bool;

    /// @note: It takes time and will return before timeout.
    auto read_image() noexcept -> std::expected<cv::Mat, std::string>;
    auto read_image_with_timestamp() noexcept -> std::expected<Image, std::string>;

    /// Alias
    ///
    auto initialize(const Config& config) noexcept -> std::expected<void, std::string> {
        configure(config);
        return connect();
    }
    auto deinitialize() noexcept -> std::expected<void, std::string> { return disconnect(); }

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace hikcamera
