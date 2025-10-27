#pragma once
#include "hikcamera/capturer.hpp"

#include "errors.hpp"
#include "utility.hpp"

#include <expected>

using namespace hikcamera;

struct Camera::Impl final {
    using Byte = unsigned char;

    sdk::ConvertParam convert_context;
    sdk::Handler camera_handler;

    std::vector<Byte> buffer;
    std::size_t buffer_size = 0;

    unsigned int timeout_ms;

    ~Impl() noexcept { std::ignore = deinitialize(); }

    template <typename T>
    auto set(char const* key, T value) noexcept -> std::expected<void, std::string> {
        if (camera_handler == nullptr) {
            std::unexpected{"Camera has not been initialized"};
        }

        auto result = std::uint32_t{};
        auto printable = std::string{};
        /*  */ if constexpr (std::same_as<T, bool>) {
            result = MV_CC_SetBoolValue(camera_handler, key, value);
            printable = std::to_string(value);
        } else if constexpr (std::same_as<T, int>) {
            result = MV_CC_SetIntValue(camera_handler, key, value);
            printable = std::to_string(value);
        } else if constexpr (std::floating_point<T>) {
            result = MV_CC_SetFloatValue(camera_handler, key, value);
            printable = std::to_string(value);
        } else if constexpr (std::is_enum_v<T>) {
            result = MV_CC_SetEnumValue(camera_handler, key, std::to_underlying(value));
            printable = "enum underlying " + std::to_string(std::to_underlying(value));
        } else {
            static_assert(false, "Unknown type of value");
        }

        if (result != sdk::OK) {
            const auto translated = translate_error(result);
            return util::make_unexpected(
                "Failed to set '{}' with {}: {}", key, printable, translated);
        }
        return {};
    }

    auto update_convert_context(sdk::FrameOut const& info) noexcept
        -> std::expected<void, std::string_view> {

        if (!util::is_rgb_pixel_type(info.stFrameInfo.enPixelType)) {
            return std::unexpected{"Camera must has RGB channel"};
        }

        auto& frame_info = info.stFrameInfo;
        buffer_size = frame_info.nWidth * frame_info.nHeight * 3;
        buffer.reserve(buffer_size);

        convert_context.nWidth = frame_info.nWidth;
        convert_context.nHeight = frame_info.nHeight;
        convert_context.nSrcDataLen = frame_info.nFrameLen;

        convert_context.enSrcPixelType = frame_info.enPixelType;
        convert_context.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

        convert_context.pDstBuffer = buffer.data();
        convert_context.nDstBufferSize = buffer_size;

        return {};
    }

    auto generate_mat(const sdk::FrameOut& source_info) const {
        // TODO:
        // Every mat shares the same memory buffer.
        // We may need a memory pool, for async process.
        return cv::Mat{
            source_info.stFrameInfo.nHeight,
            source_info.stFrameInfo.nWidth,
            CV_8UC3,
            convert_context.pDstBuffer,
        };
    }

    auto read_image_with_expected() noexcept -> std::expected<cv::Mat, std::string> {
        auto info = sdk::FrameOut{};
        auto code = std::uint32_t{};

        if (camera_handler == nullptr)
            return std::unexpected{"Attempted to read from an initialized camera"};

        code = MV_CC_GetImageBuffer(camera_handler, &info, timeout_ms);
        if (code != sdk::OK)
            return util::make_unexpected_with_error("Failed to get image buffer", code);

        if (buffer_size == 0) {
            if (auto result = update_convert_context(info); !result) {
                return util::make_unexpected(
                    "Failed to update convert context: {}", result.error());
            }
        }

        convert_context.pSrcData = info.pBufAddr;
        code = MV_CC_ConvertPixelType(camera_handler, &convert_context);
        if (code != sdk::OK)
            return util::make_unexpected_with_error("Failed to convert image", code);

        std::ignore = MV_CC_FreeImageBuffer(camera_handler, &info);

        return generate_mat(info);
    }

    auto initialize(const Config& config) -> std::expected<std::string, std::string> {

        timeout_ms = config.timeout_ms;

        auto result = util::search_device();
        if (!result.has_value())
            return std::unexpected{result.error()};

        auto device = result.value();
        if (device == nullptr)
            return std::unexpected{"Null device was got by searching"};

        auto code = std::uint32_t{};

        // Create and open device
        if (sdk::OK != (code = MV_CC_CreateHandleWithoutLog(&camera_handler, device)))
            return util::make_unexpected_with_error("Failed to create handler", code);
        auto guard_destroy = util::scope_exit{[this] { MV_CC_DestroyHandle(camera_handler); }};

        if (sdk::OK != (code = MV_CC_OpenDevice(camera_handler)))
            return util::make_unexpected_with_error("Failed to open device", code);
        auto guard_close = util::scope_exit{[this] { MV_CC_CloseDevice(camera_handler); }};

        if (device->nTLayerType == MV_GIGE_DEVICE) {
            auto size = MV_CC_GetOptimalPacketSize(camera_handler);
            if (size <= 0)
                return std::unexpected{"Invalid packet size"};

            if (auto ret = set(sdk::key::GevSCPSPacketSize, size); !ret)
                return std::unexpected{"Failed to set packet size"};
        }

        // Fixed initialize method
        {
            if (sdk::OK != (code = MV_CC_SetBayerCvtQuality(camera_handler, 2)))
                return util::make_unexpected_with_error("Failed to set bayer cvt quality", code);

            if (auto ret = set(sdk::key::ExposureAuto, sdk::ExposureAutoMode::OFF); !ret)
                return std::unexpected{ret.error()};
        }

        // initialize using config
        {
            if (auto ret = set(sdk::key::ReverseX, config.invert_image); !ret)
                return std::unexpected{ret.error()};

            if (auto ret = set(sdk::key::ReverseY, config.invert_image); !ret)
                return std::unexpected{ret.error()};

            if (auto ret = set(sdk::key::ExposureTime, config.exposure_us); !ret)
                return std::unexpected{ret.error()};

            if (auto ret = set(sdk::key::Gain, config.gain); !ret)
                return std::unexpected{ret.error()};

            auto trigger_mode = config.trigger_mode ? sdk::TriggerMode::ON : sdk::TriggerMode::OFF;
            if (auto ret = set(sdk::key::TriggerMode, trigger_mode); !ret)
                return std::unexpected{ret.error()};

            if (config.software_sync)
                if (auto ret = set(sdk::key::TriggerSource, sdk::TriggerSource::SOFTWARE); !ret)
                    return std::unexpected{ret.error()};

            if (config.fixed_framerate) {
                if (auto ret = set(sdk::key::AcquisitionFrameRateEnable, true); !ret)
                    return std::unexpected{ret.error()};

                if (auto ret = set(sdk::key::AcquisitionFrameRate, config.framerate); !ret)
                    return std::unexpected{ret.error()};
            } else {
                if (auto ret = set(sdk::key::AcquisitionFrameRateEnable, false); !ret)
                    return std::unexpected{ret.error()};
            }
        }

        // Start grabbing image
        if (sdk::OK != (code = MV_CC_StartGrabbing(camera_handler)))
            return util::make_unexpected_with_error("Failed to start grabbing", code);

        guard_destroy.release();
        guard_close.release();

        return util::make_information(*device);
    }

    auto deinitialize() const noexcept -> std::expected<void, std::string> {
        if (auto ret = MV_CC_StopGrabbing(camera_handler); ret != sdk::OK)
            return util::make_unexpected_with_error("Failed to stop grabbing:", ret);

        if (auto ret = MV_CC_CloseDevice(camera_handler); ret != sdk::OK)
            return util::make_unexpected_with_error("Failed to close device:", ret);

        if (auto ret = MV_CC_DestroyHandle(camera_handler); ret != sdk::OK)
            return util::make_unexpected_with_error("Failed to destory handle:", ret);

        return {};
    }
};
