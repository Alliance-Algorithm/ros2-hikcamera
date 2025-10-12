#include "capturer.impl.hpp"

auto Camera::initialize(Profile const& profile) noexcept
    -> std::expected<std::string, std::string> {

    if (auto result = utility::search_device())
        return pimpl->initialize(*result.value(), profile);
    else
        return std::unexpected{result.error()};
}

auto Camera::reset_connection() noexcept -> void {}

auto Camera::get_size() const noexcept -> std::expected<cv::Size2i, std::string_view> {
    if (pimpl->buffer_size == 0) {
        return std::unexpected{"Camera has not been initialized, failed to query size"};
    }
    const auto& context = pimpl->convert_context;
    return cv::Size2i{context.nWidth, context.nHeight};
}

auto Camera::read_image_with_expected() noexcept -> std::expected<cv::Mat, std::string> {}

/// @throw std::runtime_error
auto Camera::read_image_with_exception() -> cv::Mat {}

Camera::Camera() noexcept
    : pimpl{std::make_unique<Impl>()} {}

Camera::~Camera() noexcept = default;
