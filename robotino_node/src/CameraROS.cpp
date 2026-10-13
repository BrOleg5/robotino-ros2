/*
 * CameraROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 13.10.2026
 *      Author: BrOleg5
 */
#include <string>

#include "CameraROS.h"
#include "camera_info_manager/version.h"
#include "image_transport/image_transport.hpp"
#include "image_transport/version.h"
#include "sensor_msgs/fill_image.hpp"

CameraROS::CameraROS(rclcpp::Node* parent_node_ptr)
    : logger_(parent_node_ptr->get_logger()), img_msg_() {
    // image_transport and camera_info_manager 6.4 (Lyrical) deprecate rmw_qos_profile_t overloads
#if IMAGE_TRANSPORT_VERSION_GTE(6, 4, 0)
    camera_pub_ = image_transport::create_camera_publisher(*parent_node_ptr, "image_raw",
                                                           rclcpp::SystemDefaultsQoS());
#else
    camera_pub_ = image_transport::create_camera_publisher(parent_node_ptr, "image_raw");
#endif
    clock_ptr_ = parent_node_ptr->get_clock();

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

    img_msg_.header.frame_id = "camera_optical_frame";
}

void CameraROS::configure(const std::string& frame_id,
                          const std::string& camera_name,
                          const std::string& camera_info_url) {
    img_msg_.header.frame_id = frame_id;
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
    // width == 0 marks a JPEG image, it is passed undecoded only if JPEG decoding is disabled
    if (width == 0 || height == 0) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, 5000, "Received image is not decoded, skipping it");
        return;
    }
    if (dataSize < step * height) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, 5000, "Received image is truncated: %u bytes, expected %u",
                             dataSize, step * height);
        return;
    }

    // Build the Image msg
    img_msg_.header.stamp = clock_ptr_->now();
    sensor_msgs::fillImage(img_msg_, sensor_msgs::image_encodings::RGB8, height, width, step, data);

    // Build the CameraInfo msg, fall back to an uncalibrated one if calibration doesn't match image size
    sensor_msgs::msg::CameraInfo caminfo_msg = caminfo_manager_->getCameraInfo();
    if (caminfo_msg.width != width || caminfo_msg.height != height) {
        if (caminfo_manager_->isCalibrated()) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, 5000,
                                 "Calibration resolution %ux%u doesn't match image resolution %ux%u",
                                 caminfo_msg.width, caminfo_msg.height, width, height);
        }
        caminfo_msg = sensor_msgs::msg::CameraInfo();
        caminfo_msg.width = width;
        caminfo_msg.height = height;
    }
    caminfo_msg.header = img_msg_.header;

    // Publish the Image & CameraInfo msgs
    camera_pub_.publish(img_msg_, caminfo_msg);
}

void CameraROS::settingsChangedEvent(unsigned int width, unsigned int height, const char* format) {
    RCLCPP_INFO(logger_, "Camera settings: %ux%u, format %s", width, height, format);
}
