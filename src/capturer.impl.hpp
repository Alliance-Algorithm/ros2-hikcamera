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

    std::chrono::milliseconds timeout;

    auto update_convert_context(sdk::FrameOut const& info) noexcept
        -> std::expected<void, std::string_view> {

        if (!util::is_rgb_pixel_type(info.stFrameInfo.enPixelType)) {
            return std::unexpected{"Camera must has RGB channel"};
        }

        auto& frame_info = info.stFrameInfo;
        buffer_size = frame_info.nWidth * frame_info.nHeight * 3;
        buffer.resize(buffer_size, 0);

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
            source_info.stFrameInfo.nWidth,
            source_info.stFrameInfo.nHeight,
            CV_8UC3,
            convert_context.pDstBuffer,
        };
    }

    auto read_image_with_expected() noexcept -> std::expected<cv::Mat, std::string> {
        auto source_info = sdk::FrameOut{};
        auto ret = std::uint32_t{};

        ret = MV_CC_GetImageBuffer(camera_handler, &source_info, timeout.count());
        if (ret != MV_OK)
            return util::make_unexpected_with_error("Image acquisition timeout:", ret);

        if (buffer_size == 0) {
            if (auto result = update_convert_context(source_info); !result) {
                return util::make_unexpected(
                    "Failed to update convert context: {}", result.error());
            }
        }

        convert_context.pSrcData = source_info.pBufAddr;
        ret = MV_CC_ConvertPixelType(camera_handler, &convert_context);
        if (ret != MV_OK)
            return util::make_unexpected_with_error("Failed to convert image:", ret);

        ret = MV_CC_FreeImageBuffer(camera_handler, &source_info);

        return generate_mat(source_info);
    }

    template <typename T>
    auto set(char const* key, T value) noexcept -> std::expected<void, std::string> {
        if (camera_handler == nullptr) {
            std::unexpected{"Camera has not been initialized"};
        }

        auto ret = std::uint32_t{};
        /*  */ if constexpr (std::same_as<T, bool>) {
            ret = MV_CC_SetBoolValue(camera_handler, key, value);
        } else if constexpr (std::same_as<T, int>) {
            ret = MV_CC_SetIntValue(camera_handler, key, value);
        } else if constexpr (std::same_as<T, float>) {
            ret = MV_CC_SetFloatValue(camera_handler, key, value);
        } else if constexpr (std::is_enum_v<T>) {
            ret = MV_CC_SetEnumValue(camera_handler, key, value);
        } else {
            static_assert(false, "Unknown type of value");
        }

        if (ret != MV_OK) {
            return util::make_unexpected("Failed to set '{}': {}", key, translate_error(ret));
        }
        return {};
    }

    auto initialize(const sdk::DeviceInfo& device, const Config& config)
        -> std::expected<std::string, std::string> {
        auto code = std::uint32_t{};

        // Create and open device
        if (MV_OK != (code = MV_CC_CreateHandleWithoutLog(&camera_handler, &device)))
            return util::make_unexpected_with_error("Failed to create handler: {}", code);
        auto guard_destroy = util::scope_exit{[this] { MV_CC_DestroyHandle(camera_handler); }};

        if (MV_OK != (code = MV_CC_OpenDevice(camera_handler)))
            return util::make_unexpected_with_error("Failed to open device: {}", code);
        auto guard_close = util::scope_exit{[this] { MV_CC_CloseDevice(camera_handler); }};

        if (device.nTLayerType == MV_GIGE_DEVICE) {
            auto size = MV_CC_GetOptimalPacketSize(camera_handler);
            if (size <= 0)
                return std::unexpected{"Invalid packet size"};

            if (auto ret = set(sdk::key::GevSCPSPacketSize, size); !ret)
                return std::unexpected{"Failed to set packet size"};
        }

        // Fixed initialize method
        {
            if (MV_OK != (code = MV_CC_SetBayerCvtQuality(camera_handler, 2)))
                return util::make_unexpected_with_error("Failed to set bayer cvt quality", code);

            if (auto ret = set(sdk::key::ExposureAuto, MV_EXPOSURE_AUTO_MODE_OFF))
                return std::unexpected{ret.error()};

            if (auto ret = set(sdk::key::AcquisitionFrameRateEnable, false))
                return std::unexpected{ret.error()};
        }

        // initialize using config
        {
            if (auto ret = set(sdk::key::ReverseX, config.invert_image))
                return std::unexpected{ret.error()};

            if (auto ret = set(sdk::key::ReverseY, config.invert_image))
                return std::unexpected{ret.error()};

            if (auto ret = set(sdk::key::ExposureTime, config.exposure_ms))
                return std::unexpected{ret.error()};

            if (auto ret = set(sdk::key::Gain, config.gain))
                return std::unexpected{ret.error()};

            auto trigger_mode = config.trigger_mode ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF;
            if (auto ret = set(sdk::key::TriggerMode, trigger_mode); !ret)
                return std::unexpected{ret.error()};

            if (config.software_sync)
                if (auto ret = set(sdk::key::TriggerSource, MV_TRIGGER_SOURCE_SOFTWARE); !ret)
                    std::unexpected{ret.error()};
        }

        if (MV_OK != (code = MV_CC_StartGrabbing(camera_handler)))
            return util::make_unexpected_with_error("Failed to start grabbing", code);

        guard_destroy.release();
        guard_close.release();

        return util::make_information(device);
    }
};
