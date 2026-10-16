/*
 * CameraROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 13.10.2026
 *      Author: BrOleg5
 */
#include <algorithm>
#include <stdexcept>
#include <string>

#include "CameraROS.h"
#include "camera_info_manager/version.h"
#include "sensor_msgs/image_encodings.hpp"

namespace {
// Format string of compressed_image_transport: original encoding; JPEG decodes to BGR with OpenCV
const char* const kJpegFormat = "rgb8; jpeg compressed bgr8";
// Fast integer DCT: noticeably faster with negligible quality loss for camera images
const int kTurboJpegFlags = TJFLAG_FASTDCT;
}  // namespace

CameraROS::CameraROS(rclcpp::Node* parent_node_ptr)
    : logger_(parent_node_ptr->get_logger()),
      frame_id_("camera_optical_frame"),
      decompressor_(tjInitDecompress()),
      compressor_(tjInitCompress()) {
    if (decompressor_ == nullptr || compressor_ == nullptr) {
        throw std::runtime_error(std::string("Cannot initialize TurboJPEG: ") + tjGetErrorStr2(nullptr));
    }

    // Same topic names and QoS as image_transport::CameraPublisher with base topic image_raw
    image_pub_ = parent_node_ptr->create_publisher<sensor_msgs::msg::Image>("image_raw",
                                                                            rclcpp::SystemDefaultsQoS());
    compressed_pub_ = parent_node_ptr->create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_raw/compressed", rclcpp::SystemDefaultsQoS());
    caminfo_pub_ = parent_node_ptr->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info",
                                                                                  rclcpp::SystemDefaultsQoS());
    clock_ptr_ = parent_node_ptr->get_clock();

    // camera_info_manager 6.4 (Lyrical) deprecates the rmw_qos_profile_t overload.
    // Service ~/set_camera_info, as in Jazzy (Lyrical puts it into the root namespace by default)
#if CAMERA_INFO_MANAGER_VERSION_GTE(6, 4, 0)
    caminfo_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        parent_node_ptr->get_node_base_interface(), parent_node_ptr->get_node_services_interface(),
        parent_node_ptr->get_node_logging_interface(), "camera", "", rclcpp::SystemDefaultsQoS(), "~");
#else
    caminfo_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        parent_node_ptr->get_node_base_interface(), parent_node_ptr->get_node_services_interface(),
        parent_node_ptr->get_node_logging_interface(), "camera", "", rmw_qos_profile_default);
#endif

    // Receive JPEG images as is, they are decoded only when needed
    this->setJPGDecodingEnabled(false);
}

CameraROS::~CameraROS() {
    tjFree(jpeg_buf_);
    tjDestroy(compressor_);
    tjDestroy(decompressor_);
}

void CameraROS::configure(const std::string& frame_id,
                          const std::string& camera_name,
                          const std::string& camera_info_url,
                          int jpeg_quality) {
    frame_id_ = frame_id;
    jpeg_quality_ = std::clamp(jpeg_quality, 1, 100);
    if (!caminfo_manager_->setCameraName(camera_name)) {
        RCLCPP_WARN(logger_, "Invalid camera name '%s', using 'camera'", camera_name.c_str());
    }
    if (!caminfo_manager_->validateURL(camera_info_url)) {
        RCLCPP_WARN(logger_, "Invalid camera_info_url '%s'", camera_info_url.c_str());
    }
    caminfo_manager_->loadCameraInfo(camera_info_url);
    if (!caminfo_manager_->isCalibrated()) {
        RCLCPP_WARN(logger_, "Camera is not calibrated, camera_info contains image size only");
    }
}

void CameraROS::imageReceivedEvent(const unsigned char* data,
                                   unsigned int dataSize,
                                   unsigned int width,
                                   unsigned int height,
                                   unsigned int step) {
    std_msgs::msg::Header header;
    header.stamp = clock_ptr_->now();
    header.frame_id = frame_id_;

    // width == 0 marks a JPEG image
    if (width == 0) {
        int jpeg_width = 0;
        int jpeg_height = 0;
        if (processJpegImage(header, data, dataSize, jpeg_width, jpeg_height)) {
            publishCameraInfo(header, jpeg_width, jpeg_height);
        }
    } else if (processRawImage(header, data, dataSize, width, height, step)) {
        publishCameraInfo(header, width, height);
    }
}

bool CameraROS::processJpegImage(const std_msgs::msg::Header& header,
                                 const unsigned char* data,
                                 unsigned int dataSize,
                                 int& width,
                                 int& height) {
    int subsamp = 0;
    int colorspace = 0;
    if (tjDecompressHeader3(decompressor_, data, dataSize, &width, &height, &subsamp, &colorspace) != 0) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, 5000, "Received invalid JPEG image: %s",
                             tjGetErrorStr2(decompressor_));
        return false;
    }

    if (compressed_pub_->get_subscription_count() > 0) {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header = header;
        msg->format = kJpegFormat;
        msg->data.assign(data, data + dataSize);
        compressed_pub_->publish(std::move(msg));
    }

    if (image_pub_->get_subscription_count() > 0) {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header = header;
        msg->width = width;
        msg->height = height;
        msg->encoding = sensor_msgs::image_encodings::RGB8;
        msg->is_bigendian = false;
        msg->step = width * tjPixelSize[TJPF_RGB];
        msg->data.resize(msg->step * height);
        // Decode directly into the message buffer
        if (tjDecompress2(decompressor_, data, dataSize, msg->data.data(), width, msg->step, height, TJPF_RGB,
                          kTurboJpegFlags) != 0) {
            // Warnings (e.g. truncated data) still produce an image
            if (tjGetErrorCode(decompressor_) == TJERR_FATAL) {
                RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, 5000, "Cannot decode JPEG image: %s",
                                     tjGetErrorStr2(decompressor_));
                return true;
            }
            RCLCPP_DEBUG(logger_, "JPEG decoding warning: %s", tjGetErrorStr2(decompressor_));
        }
        image_pub_->publish(std::move(msg));
    }
    return true;
}

bool CameraROS::processRawImage(const std_msgs::msg::Header& header,
                                const unsigned char* data,
                                unsigned int dataSize,
                                unsigned int width,
                                unsigned int height,
                                unsigned int step) {
    if (height == 0 || dataSize < step * height) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, 5000, "Received image is truncated: %u bytes, expected %u",
                             dataSize, step * height);
        return false;
    }

    if (image_pub_->get_subscription_count() > 0) {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header = header;
        msg->width = width;
        msg->height = height;
        msg->encoding = sensor_msgs::image_encodings::RGB8;
        msg->is_bigendian = false;
        msg->step = step;
        msg->data.assign(data, data + step * height);
        image_pub_->publish(std::move(msg));
    }

    if (compressed_pub_->get_subscription_count() > 0) {
        // Preallocated buffer of the maximal JPEG size: no reallocation per image
        unsigned long buf_size = tjBufSize(width, height, TJSAMP_420);
        if (buf_size > jpeg_buf_size_) {
            tjFree(jpeg_buf_);
            jpeg_buf_ = tjAlloc(buf_size);
            jpeg_buf_size_ = buf_size;
        }
        unsigned long jpeg_size = jpeg_buf_size_;
        if (tjCompress2(compressor_, data, width, step, height, TJPF_RGB, &jpeg_buf_, &jpeg_size, TJSAMP_420,
                        jpeg_quality_, kTurboJpegFlags | TJFLAG_NOREALLOC) != 0) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, 5000, "Cannot encode JPEG image: %s",
                                 tjGetErrorStr2(compressor_));
        } else {
            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            msg->header = header;
            msg->format = kJpegFormat;
            msg->data.assign(jpeg_buf_, jpeg_buf_ + jpeg_size);
            compressed_pub_->publish(std::move(msg));
        }
    }
    return true;
}

void CameraROS::publishCameraInfo(const std_msgs::msg::Header& header, unsigned int width, unsigned int height) {
    // Fall back to an uncalibrated CameraInfo if calibration doesn't match image size
    auto msg = std::make_unique<sensor_msgs::msg::CameraInfo>(caminfo_manager_->getCameraInfo());
    if (msg->width != width || msg->height != height) {
        if (caminfo_manager_->isCalibrated()) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, 5000,
                                 "Calibration resolution %ux%u doesn't match image resolution %ux%u", msg->width,
                                 msg->height, width, height);
        }
        *msg = sensor_msgs::msg::CameraInfo();
        msg->width = width;
        msg->height = height;
    }
    msg->header = header;
    caminfo_pub_->publish(std::move(msg));
}

void CameraROS::settingsChangedEvent(unsigned int width, unsigned int height, const char* format) {
    RCLCPP_INFO(logger_, "Camera settings: %ux%u, format %s", width, height, format);
}
