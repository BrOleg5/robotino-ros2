/*
 * CameraROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 19.07.2024
 *      Author: BrOleg5
 */
#include <string>

#include "CameraROS.h"
#include "sensor_msgs/fill_image.hpp"

CameraROS::CameraROS(rclcpp::Node* parent_node_ptr) : img_msg_(), caminfo_msg_() {
    streaming_pub_ = parent_node_ptr->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    caminfor_pub_ = parent_node_ptr->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    clock_ptr_ = parent_node_ptr->get_clock();

    img_msg_.header.frame_id = "camera_link";
    caminfo_msg_.header.frame_id = "camera_link";
}

void CameraROS::imageReceivedEvent(const unsigned char* data,
                                   unsigned int dataSize,
                                   unsigned int width,
                                   unsigned int height,
                                   unsigned int step) {
    // Build the Image msg
    img_msg_.header.stamp = clock_ptr_->now();
    sensor_msgs::fillImage(img_msg_, sensor_msgs::image_encodings::RGB8, height, width, step, data);

    caminfo_msg_.height = height;
    caminfo_msg_.width = width;

    // Publish the Image & CameraInfo msgs
    streaming_pub_->publish(img_msg_);
    caminfor_pub_->publish(caminfo_msg_);
}
