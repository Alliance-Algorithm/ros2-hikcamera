#pragma once
#include <chrono>
#include <expected>
#include <opencv2/core/mat.hpp>

namespace hikcamera {
using Milli = std::chrono::milliseconds;

struct Config final {

    static constexpr auto kMaxGain = float{16.9807};

    float exposure_ms = 2.;
    float gain = kMaxGain;
    float frame_rate = 80.;

    Milli timeout{2};

    bool trigger_mode = false;
    bool invert_image = false;
    bool software_sync = false;
};

class Camera final {
public:
    // @note: Default it enough for most situation
    auto initialize(const Config& config = {}) noexcept //
        -> std::expected<std::string, std::string>;

    auto initialized() const noexcept -> bool;

    auto reset_connection() noexcept -> void;

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
