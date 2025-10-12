#pragma once
#include "hikcamera/capturer.hpp"

#include "errors.hpp"
#include "utility.hpp"

#include <expected>

using namespace hikcamera;

struct Camera::Impl final {
    sdk::ConvertParam convert_context;
    sdk::Handler camera_handler;

    std::vector<unsigned char> buffer;
    std::size_t buffer_size = 0;

    std::chrono::milliseconds timeout;

    auto read_image_with_expected() noexcept -> std::expected<cv::Mat, std::string> {
        auto source_info = sdk::FrameOut{};
        auto ret = std::uint32_t{};

        ret = MV_CC_GetImageBuffer(camera_handler, &source_info, timeout.count());
        if (ret != MV_OK)
            return utility::make_unexpected("Image acquisition timeout:", ret);

        if (buffer_size == 0) {
            auto result = update_convert_context(source_info);
            if (!result) {
                return utility::make_unexpected(
                    "Failed to update convert context: {}", result.error());
            }
        }

        convert_context.pSrcData = source_info.pBufAddr;
        ret = MV_CC_ConvertPixelType(camera_handler, &convert_context);
        if (ret != MV_OK)
            return utility::make_unexpected("Failed to convert image:", ret);

        ret = MV_CC_FreeImageBuffer(camera_handler, &source_info);

        return cv::Mat{
            source_info.stFrameInfo.nWidth,
            source_info.stFrameInfo.nHeight,
            CV_8UC3,
            convert_context.pDstBuffer,
        };
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
            return utility::make_unexpected("Failed to set value:", ret);
        }
        return {};
    }

    auto update_convert_context(sdk::FrameOut const& info) noexcept
        -> std::expected<void, std::string_view> {

        if (!utility::is_rgb_pixel_type(info.stFrameInfo.enPixelType)) {
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

    auto initialize(const sdk::DeviceInfo& device, const Profile& profile)
        -> std::expected<std::string, std::string> {

        auto handler = sdk::Handler{};
        auto status = std::uint32_t{};

        status = MV_CC_CreateHandleWithoutLog(&handler, &device);
        if (status != MV_OK) {
            return std::unexpected{
                std::format("Failed to create handler: {}", translate_error(status)),
            };
        }
        utility::DelayRun remove_handler{[handler] { MV_CC_DestroyHandle(handler); }};

        status = MV_CC_OpenDevice(handler);
        if (status != MV_OK) {
            return std::unexpected{
                std::format("Failed to open device: {}", translate_error(status)),
            };
        }
        utility::DelayRun close_device{[handler] { MV_CC_CloseDevice(handler); }};

        if (device.nTLayerType == MV_GIGE_DEVICE) {
            auto size = MV_CC_GetOptimalPacketSize(handler);
            if (size <= 0)
                return std::unexpected{"Invalid packet size"};

            if (auto ret = set<int>("GevSCPSPacketSize", size); !ret) {}
        }

        remove_handler.cancel();
        close_device.cancel();
        camera_handler = handler;
        return utility::make_information(device);
    }
};
