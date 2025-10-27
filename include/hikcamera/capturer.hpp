#pragma once
#include <expected>
#include <opencv2/core/mat.hpp>

namespace hikcamera {
static constexpr auto kMaxGain = float{16.9807};

struct Config final {

    unsigned int timeout_ms = 2000;

    float exposure_us = 2000.;
    float framerate = 80;
    float gain = kMaxGain;

    bool invert_image = false;
    bool software_sync = false;

    bool trigger_mode = false;
    bool fixed_framerate = true;
};

class Camera final {
public:
    // @note: Default it enough for most situation
    auto initialize(const Config& config = {}) noexcept //
        -> std::expected<std::string, std::string>;

    auto deinitialize() noexcept -> std::expected<void, std::string>;

    auto initialized() const noexcept -> bool;

    auto get_size() const noexcept -> std::expected<cv::Size2i, std::string_view>;

    auto read_image() noexcept -> std::expected<cv::Mat, std::string>;

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
