#pragma once
#include "errors.hpp"
#include "sdk/include/MvCameraControl.h"

#include <experimental/scope>

#include <cstring>
#include <expected>
#include <format>
#include <string_view>

namespace hikcamera::sdk {
using Handler = void*;

using DeviceInfo = MV_CC_DEVICE_INFO;
using DeviceInfoList = MV_CC_DEVICE_INFO_LIST;

using FrameOut = MV_FRAME_OUT;

using ConvertParam = MV_CC_PIXEL_CONVERT_PARAM;

namespace key {
constexpr auto GevSCPSPacketSize = "GevSCPSPacketSize";
constexpr auto ExposureAuto = "ExposureAuto";
constexpr auto AcquisitionFrameRateEnable = "AcquisitionFrameRateEnable";
constexpr auto ReverseX = "ReverseX";
constexpr auto ReverseY = "ReverseY";
constexpr auto ExposureTime = "ExposureTime";
constexpr auto Gain = "Gain";
constexpr auto TriggerMode = "TriggerMode";
constexpr auto TriggerSource = "TriggerSource";
} // namespace key

} // namespace hikcamera::sdk

namespace hikcamera::util {

template <typename Ef>
using scope_exit = std::experimental::scope_exit<Ef>;

template <typename... Args>
constexpr auto make_unexpected(std::format_string<Args...> fmt, Args&&... args) noexcept {
    return std::unexpected{std::format(fmt, std::forward<Args>(args)...)};
}
constexpr auto make_unexpected_with_error(std::string_view msg, std::uint32_t error) noexcept {
    return std::unexpected{std::format("{}: {}", msg, translate_error(error))};
}

constexpr auto is_rgb_pixel_type(MvGvspPixelType type) noexcept -> bool {
    switch (type) {
    case PixelType_Gvsp_BGR8_Packed:
    case PixelType_Gvsp_YUV422_Packed:
    case PixelType_Gvsp_YUV422_YUYV_Packed:
    case PixelType_Gvsp_BayerGR8:
    case PixelType_Gvsp_BayerRG8:
    case PixelType_Gvsp_BayerGB8:
    case PixelType_Gvsp_BayerBG8:
    case PixelType_Gvsp_BayerGB10:
    case PixelType_Gvsp_BayerGB10_Packed:
    case PixelType_Gvsp_BayerBG10:
    case PixelType_Gvsp_BayerBG10_Packed:
    case PixelType_Gvsp_BayerRG10:
    case PixelType_Gvsp_BayerRG10_Packed:
    case PixelType_Gvsp_BayerGR10:
    case PixelType_Gvsp_BayerGR10_Packed:
    case PixelType_Gvsp_BayerGB12:
    case PixelType_Gvsp_BayerGB12_Packed:
    case PixelType_Gvsp_BayerBG12:
    case PixelType_Gvsp_BayerBG12_Packed:
    case PixelType_Gvsp_BayerRG12:
    case PixelType_Gvsp_BayerRG12_Packed:
    case PixelType_Gvsp_BayerGR12:
    case PixelType_Gvsp_BayerGR12_Packed: return true;
    default: return false;
    }
}

constexpr auto compare(sdk::DeviceInfo const& info, std::string_view other) noexcept -> bool {

    unsigned char const* raw_name;
    const auto& special = info.SpecialInfo;

    switch (info.nTLayerType) {
    case MV_GIGE_DEVICE: {
        raw_name = special.stGigEInfo.chUserDefinedName;
        break;
    }
    case MV_USB_DEVICE: {
        raw_name = special.stUsb3VInfo.chUserDefinedName;
        break;
    }
    default: return false;
    }

    if (const auto device_name = reinterpret_cast<char const*>(raw_name)) {
        return other == device_name;
    } else {
        return false;
    }
}

constexpr auto search_device(std::string_view custom_definition = {}) noexcept
    -> std::expected<sdk::DeviceInfo*, std::string> {

    auto devices = sdk::DeviceInfoList{};
    std::memset(&devices, 0, sizeof(devices));

    const auto result = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
    if (result != MV_OK) {
        const auto msg = translate_error(result);
        return std::unexpected{std::format("Failed to enum device: {}", msg)};
    }

    const auto [device_num, device_infos] = devices;
    if (device_num == 0) {
        return std::unexpected{"No device was found"};
    }
    if (custom_definition.empty()) {
        if (device_num == 1 && device_infos[0] != nullptr) {
            return device_infos[0];
        } else {
            return std::unexpected{std::format("{} devices were found", device_num)};
        }
    } else {
        for (auto index = 0; index < device_num; index++) {
            auto* info_ptr = device_infos[index];
            if (info_ptr == nullptr)
                continue;
            if (compare(*info_ptr, custom_definition)) {
                return info_ptr;
            }
        }
        return std::unexpected{
            std::format("No device matches the custom name among {} devices", device_num)};
    }
    return std::unexpected{"Unreachable result"};
}

constexpr auto make_information(const sdk::DeviceInfo& info) noexcept -> std::string {
    auto result = std::string{};

    if (info.nTLayerType == MV_GIGE_DEVICE) {
        const auto& gige = info.SpecialInfo.stGigEInfo;

        auto ip1 = (gige.nCurrentIp & 0xff000000) >> 24;
        auto ip2 = (gige.nCurrentIp & 0x00ff0000) >> 16;
        auto ip3 = (gige.nCurrentIp & 0x0000ff00) >> 8;
        auto ip4 = (gige.nCurrentIp & 0x000000ff);

        result += std::format("Device Type: GigE\n");
        result += std::format("Device IP: {}.{}.{}.{}\n", ip1, ip2, ip3, ip4);
        result += std::format(
            "User Defined Name: {}\n", reinterpret_cast<const char*>(gige.chUserDefinedName));
    } else if (info.nTLayerType == MV_USB_DEVICE) {
        const auto& usb = info.SpecialInfo.stUsb3VInfo;

        result += std::format("Device Type: USB\n");
        result += std::format(
            "User Defined Name: {}\n", reinterpret_cast<const char*>(usb.chUserDefinedName));
        result +=
            std::format("Serial Number: {}\n", reinterpret_cast<const char*>(usb.chSerialNumber));
        result += std::format("Device Number: {}\n", usb.nDeviceNumber);
    } else {
        result += "Device Type: Unknown\n";
    }
    return result;
}

} // namespace hikcamera::util
