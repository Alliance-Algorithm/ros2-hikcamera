#pragma once
#include "sdk/include/MvErrorDefine.h"

#include <cstdint>
#include <string_view>

namespace hikcamera {
constexpr auto translate_error(std::uint32_t code) noexcept -> std::string_view {
    // clang-format off
    switch (code) {
        case MV_OK:                      return "Success, no error";

        // General error codes
        case MV_E_HANDLE:                return "Error or invalid handle";
        case MV_E_SUPPORT:               return "Function not supported";
        case MV_E_BUFOVER:               return "Buffer overflow";
        case MV_E_CALLORDER:             return "Function calling order error";
        case MV_E_PARAMETER:             return "Incorrect parameter";
        case MV_E_RESOURCE:              return "Resource allocation failed";
        case MV_E_NODATA:                return "No data available";
        case MV_E_PRECONDITION:          return "Precondition error or environment changed";
        case MV_E_VERSION:               return "Version mismatch";
        case MV_E_NOENOUGH_BUF:          return "Insufficient memory";
        case MV_E_ABNORMAL_IMAGE:        return "Abnormal or incomplete image";
        case MV_E_LOAD_LIBRARY:          return "Failed to load dynamic library";
        case MV_E_NOOUTBUF:              return "No available output buffer";
        case MV_E_ENCRYPT:               return "Encryption error";
        case MV_E_OPENFILE:              return "File open error";
        case MV_E_UNKNOW:                return "Unknown error";

        // GenICam error codes
        case MV_E_GC_GENERIC:            return "General GenICam error";
        case MV_E_GC_ARGUMENT:           return "Illegal GenICam argument";
        case MV_E_GC_RANGE:              return "GenICam value out of range";
        case MV_E_GC_PROPERTY:           return "GenICam property error";
        case MV_E_GC_RUNTIME:            return "GenICam runtime error";
        case MV_E_GC_LOGICAL:            return "GenICam logical error";
        case MV_E_GC_ACCESS:             return "GenICam node access error";
        case MV_E_GC_TIMEOUT:            return "GenICam timeout";
        case MV_E_GC_DYNAMICCAST:        return "GenICam transformation exception";
        case MV_E_GC_UNKNOW:             return "Unknown GenICam error";

        // GigE error codes
        case MV_E_NOT_IMPLEMENTED:       return "Command not supported by device";
        case MV_E_INVALID_ADDRESS:       return "Invalid target address";
        case MV_E_WRITE_PROTECT:         return "Target address is write-protected";
        case MV_E_ACCESS_DENIED:         return "Access denied to device";
        case MV_E_BUSY:                  return "Device busy or network disconnected";
        case MV_E_PACKET:                return "Network packet error";
        case MV_E_NETER:                 return "Network error";
        case MV_E_KEY_VERIFICATION:      return "Key verification error";
        case MV_E_IP_CONFLICT:           return "Device IP conflict";

        // USB error codes
        case MV_E_USB_READ:              return "USB read error";
        case MV_E_USB_WRITE:             return "USB write error";
        case MV_E_USB_DEVICE:            return "USB device exception";
        case MV_E_USB_GENICAM:           return "USB GenICam error";
        case MV_E_USB_BANDWIDTH:         return "Insufficient USB bandwidth";
        case MV_E_USB_DRIVER:            return "USB driver mismatch or not installed";
        case MV_E_USB_UNKNOW:            return "Unknown USB error";

        // Upgrade error codes
        case MV_E_UPG_FILE_MISMATCH:     return "Firmware file mismatch";
        case MV_E_UPG_LANGUSGE_MISMATCH: return "Firmware language mismatch";
        case MV_E_UPG_CONFLICT:          return "Upgrade conflict (device already upgrading)";
        case MV_E_UPG_INNER_ERR:         return "Internal device error during upgrade";
        case MV_E_UPG_UNKNOW:            return "Unknown upgrade error";

        default:                         return "Unrecognized error code";
        // clang-format on
    }
}
} // namespace hikcamera
