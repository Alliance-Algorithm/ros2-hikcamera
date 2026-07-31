#include "capturer.impl.hpp"

auto hikcamera::list_camera_names() noexcept
    -> std::expected<std::vector<std::string>, std::string> {
    return util::list_camera_names();
}

auto Camera::configure(const Config& config) noexcept -> void { pimpl->configure(config); }

auto Camera::connect() noexcept -> std::expected<void, std::string> { return pimpl->connect(); }

auto Camera::connected() const noexcept -> bool { return pimpl->camera_handler != nullptr; }

auto Camera::disconnect() noexcept -> std::expected<void, std::string> {
    return pimpl->disconnect();
}

auto Camera::read_image() noexcept -> std::expected<cv::Mat, std::string> {
    return pimpl->read_image();
}
auto Camera::read_image_with_timestamp() noexcept -> std::expected<Image, std::string> {
    return pimpl->read_image_with_timestamp();
}

Camera::Camera() noexcept
    : pimpl{std::make_unique<Impl>()} {}

Camera::~Camera() noexcept = default;
