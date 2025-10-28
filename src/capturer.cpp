#include "capturer.impl.hpp"

auto Camera::initialize(const Config& config) noexcept //
    -> std::expected<void, std::string> {
    return pimpl->initialize(config);
}

auto Camera::initialized() const noexcept -> bool { return pimpl->camera_handler != nullptr; }

auto Camera::deinitialize() noexcept -> std::expected<void, std::string> {
    return pimpl->deinitialize();
}

auto Camera::get_size() const noexcept -> std::expected<cv::Size2i, std::string_view> {
    if (pimpl->buffer_size == 0) {
        return std::unexpected{"Camera has not been initialized, failed to query size"};
    }
    const auto& context = pimpl->convert_context;
    return cv::Size2i{context.nWidth, context.nHeight};
}

auto Camera::read_image() noexcept -> std::expected<cv::Mat, std::string> {
    return pimpl->read_image_with_expected();
}

Camera::Camera() noexcept
    : pimpl{std::make_unique<Impl>()} {}

Camera::~Camera() noexcept = default;
