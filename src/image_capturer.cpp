#include "hikcamera/image_capturer.hpp"

#include <cstring>

#include <chrono>
#include <ratio>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sdk/include/MvCameraControl.h>

namespace hikcamera {

class ImageCapturer {
private:
    void* _handle                = nullptr;
    unsigned int ConvertDataSize = 0;
    unsigned char* pConvertData  = nullptr;
    MV_CC_PIXEL_CONVERT_PARAM stConvertParam;

    rclcpp::Logger logger_ = rclcpp::get_logger("hikcamera");

    bool is_same_device_name(MV_CC_DEVICE_INFO* pstMVDevInfo, const char* targetName) {
        if (nullptr == pstMVDevInfo) {
            RCLCPP_ERROR(logger_, "The Pointer of pstMVDevInfo is NULL!");
            return false;
        }
        const unsigned char* deviceName;
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            deviceName = pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName;
        } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
            deviceName = pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
        } else {
            return false;
        }

        return strcmp(reinterpret_cast<const char*>(deviceName), targetName) == 0;
    }

    bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) {
        if (nullptr == pstMVDevInfo) {
            RCLCPP_ERROR(logger_, "The Pointer of pstMVDevInfo is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
            RCLCPP_INFO(logger_, "DeviceIp: %d.%d.%d.%d", nIp1, nIp2, nIp3, nIp4);
            RCLCPP_INFO(
                logger_, "UserDefinedName: %s",
                pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
            RCLCPP_INFO(
                logger_, "UserDefinedName: %s",
                pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
            RCLCPP_INFO(
                logger_, "Serial Number: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            RCLCPP_INFO(
                logger_, "Device Number: %u", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
        } else {
            RCLCPP_ERROR(logger_, "Neither a GigE camera nor a USB camera.");
            return false;
        }

        return true;
    }

    MV_CC_DEVICE_INFO* search_camera(const char* user_defined_name) {
        MV_CC_DEVICE_INFO_LIST device_list;
        memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        unsigned int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
        if (ret != MV_OK) {
            RCLCPP_ERROR(logger_, "Failed to enum Devices. nRet [%u]", ret);
            return nullptr;
        }
        if (device_list.nDeviceNum == 0) {
            RCLCPP_ERROR(logger_, "Find No Devices.");
            return nullptr;
        }

        if (user_defined_name == nullptr) {
            if (device_list.nDeviceNum > 1) {
                RCLCPP_ERROR(
                    logger_, "Must pass in the device name because %u devices were found.",
                    device_list.nDeviceNum);
                return nullptr;
            }
            return device_list.pDeviceInfo[0];
        } else {
            for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
                if (is_same_device_name(device_list.pDeviceInfo[i], user_defined_name))
                    return device_list.pDeviceInfo[i];
            }
            RCLCPP_ERROR(
                logger_, "%u devices was found, but no device matches the name passed in: %s",
                device_list.nDeviceNum, user_defined_name);
            return nullptr;
        }
    }

    template <typename Func>
    struct FinalAction {
        FinalAction(Func func)
            : clean_{func}
            , enabled_(true) {}

        FinalAction(const FinalAction&)            = delete;
        FinalAction& operator=(const FinalAction&) = delete;
        FinalAction(FinalAction&&)                 = delete;
        FinalAction& operator=(FinalAction&&)      = delete;

        ~FinalAction() {
            if (enabled_)
                clean_();
        }

        void disable() { enabled_ = false; };

    private:
        Func clean_;
        bool enabled_;
    };

#define SDK_RET_ASSERT(ret, message)                          \
    do {                                                      \
        if (ret != MV_OK) {                                   \
            RCLCPP_ERROR(logger_, message " nRet [%u]", ret); \
            return false;                                     \
        }                                                     \
    } while (false)

    bool init_camera(MV_CC_DEVICE_INFO& device_info) {
        auto pDeviceInfo = &device_info;

        unsigned int ret;

        ret = MV_CC_CreateHandle(&_handle, pDeviceInfo);
        SDK_RET_ASSERT(ret, "Failed to create handle.");
        FinalAction destroy_handle{[this]() { MV_CC_DestroyHandle(_handle); }};

        ret = MV_CC_OpenDevice(_handle);
        SDK_RET_ASSERT(ret, "Failed to open device.");
        FinalAction close_device{[this]() { MV_CC_CloseDevice(_handle); }};

        if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            int nPacketSize = MV_CC_GetOptimalPacketSize(_handle);
            if (nPacketSize <= 0) {
                RCLCPP_ERROR(logger_, "Get invaild packet Size: %d", nPacketSize);
                return false;
            }

            ret = MV_CC_SetIntValue(_handle, "GevSCPSPacketSize", nPacketSize);
            SDK_RET_ASSERT(ret, "Failed to set packet Size.");
        }

        ret = MV_CC_SetEnumValue(_handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
        SDK_RET_ASSERT(ret, "Failed to set trigger Mode.");

        if (false) {
            ret = MV_CC_SetBoolValue(_handle, "ReverseX", true);
            SDK_RET_ASSERT(ret, "Failed to set reverse x.");
            ret = MV_CC_SetBoolValue(_handle, "ReverseY", true);
            SDK_RET_ASSERT(ret, "Failed to set reverse y.");
        } else {
            ret = MV_CC_SetBoolValue(_handle, "ReverseX", false);
            SDK_RET_ASSERT(ret, "Failed to set reverse x.");
            ret = MV_CC_SetBoolValue(_handle, "ReverseY", false);
            SDK_RET_ASSERT(ret, "Failed to set reverse y.");
        }

        ret = MV_CC_SetEnumValue(_handle, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
        SDK_RET_ASSERT(ret, "Failed to set auto exposure.");

        ret = MV_CC_SetFloatValue(_handle, "ExposureTime", 3000);
        SDK_RET_ASSERT(ret, "Failed to set exposure time.");

        ret = MV_CC_SetFloatValue(_handle, "Gain", 10);
        SDK_RET_ASSERT(ret, "Failed to set gain.");

        ret = MV_CC_SetBoolValue(_handle, "AcquisitionFrameRateEnable", false);
        SDK_RET_ASSERT(ret, "Failed to set acquisition frame rate enable.");

        ret = MV_CC_SetBayerCvtQuality(_handle, 2);
        SDK_RET_ASSERT(ret, "Failed to set bayer cvt quality.");

        ret = MV_CC_StartGrabbing(_handle);
        SDK_RET_ASSERT(ret, "Failed to start grabbing.");

        destroy_handle.disable();
        close_device.disable();
        return true;
    }

    bool IsRGBCamera(MvGvspPixelType enType) {
        switch (enType) {
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

    void UnloadCamera() {
        unsigned int nRet;
        nRet = MV_CC_StopGrabbing(_handle);
        if (MV_OK != nRet)
            RCLCPP_ERROR(logger_, "Failed to stop grabbing. nRet [%u]", nRet);

        nRet = MV_CC_CloseDevice(_handle);
        if (MV_OK != nRet)
            RCLCPP_ERROR(logger_, "Failed to close device. nRet [%u]", nRet);

        nRet = MV_CC_DestroyHandle(_handle);
        if (MV_OK != nRet)
            RCLCPP_ERROR(logger_, "Failed to destroy handle. nRet [%u]", nRet);
    }

public:
    ImageCapturer(const char* userDefinedName = nullptr) {
        auto device_info = search_camera(userDefinedName);
        if (!device_info)
            throw std::runtime_error{"Failed to search camera, see log for details."};

        if (!init_camera(*device_info))
            throw std::runtime_error{"Failed to init camera, see log for details."};
    }
    ImageCapturer(const ImageCapturer&)            = delete;
    ImageCapturer& operator=(const ImageCapturer&) = delete;

    ~ImageCapturer() {
        UnloadCamera();
        if (pConvertData != nullptr)
            delete[] pConvertData;
    }

    cv::Mat Read(std::chrono::duration<unsigned int, std::micro> timeout) {
        MV_FRAME_OUT stImageInfo;

        unsigned int ret = MV_CC_GetImageBuffer(_handle, &stImageInfo, timeout.count());
        if (ret != MV_OK) {
            RCLCPP_ERROR(logger_, "Image getting timeout. nRet [%u]", ret);
            throw std::runtime_error{"Failed to convert pixel type"};
        }
        // 调用海康SDK进行Bayer2BGR，一定要设置BayerCvtQuality，其在windows和linux下的默认值不同
        // 这里只考虑相机传入的每帧图像大小、格式不变的情况
        if (pConvertData == nullptr) {
            if (!IsRGBCamera(stImageInfo.stFrameInfo.enPixelType))
                throw std::runtime_error{"RGB camera needed!"};

            ConvertDataSize = stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight * 3;
            pConvertData    = new unsigned char[ConvertDataSize];

            stConvertParam.nWidth      = stImageInfo.stFrameInfo.nWidth;    // image width
            stConvertParam.nHeight     = stImageInfo.stFrameInfo.nHeight;   // image height
            stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen; // input data size
            stConvertParam.enSrcPixelType =
                stImageInfo.stFrameInfo.enPixelType;                        // input pixel format
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;     // output pixel format
            stConvertParam.pDstBuffer     = pConvertData;                   // output data buffer
            stConvertParam.nDstBufferSize = ConvertDataSize;                // output buffer size
        }

        stConvertParam.pSrcData = stImageInfo.pBufAddr;                     // input data buffer

        ret = MV_CC_ConvertPixelType(_handle, &stConvertParam);
        if (ret != MV_OK) {
            RCLCPP_ERROR(logger_, "Failed to convert pixel type. nRet [%u]", ret);
            throw std::runtime_error{"Failed to convert pixel type"};
        }

        // 注意：这是cv::Mat的一个特殊构造函数，Mat指向的图像内容不会随Mat的析构被其自动析构，处理不当会造成内存泄露
        cv::Mat img{
            stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC3,
            stConvertParam.pDstBuffer};

        // 这里不涉及堆内存释放
        MV_CC_FreeImageBuffer(_handle, &stImageInfo);

        return img;
    }
};

} // namespace hikcamera

void tttest() {
    using namespace std::chrono_literals;

    hikcamera::ImageCapturer capturer;
    capturer.Read(5s);
}