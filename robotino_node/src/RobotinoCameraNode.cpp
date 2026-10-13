/*
 * RobotinoCameraNode.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 13.10.2026
 *      Author: BrOleg5
 */
#include <string>
#include <chrono>

#include "RobotinoCameraNode.h"

RobotinoCameraNode::RobotinoCameraNode() : Node("camera"), com_(this), camera_(this) {
    this->declare_parameter("hostname", "192.168.0.1");
    this->declare_parameter("timer_period_ms", 20);
    this->declare_parameter("camera_number", 0);
    this->declare_parameter("frame_id", "camera_optical_frame");
    this->declare_parameter("camera_name", "camera");
    this->declare_parameter("camera_info_url", "");
    // Requested resolution, 0 keeps the current camera settings
    this->declare_parameter("image_width", 0);
    this->declare_parameter("image_height", 0);
    // "mjpg" (compressed transfer, decoded by Robotino API2) or "raw"
    this->declare_parameter("image_format", "mjpg");

    camera_.configure(this->get_parameter("frame_id").as_string(),
                      this->get_parameter("camera_name").as_string(),
                      this->get_parameter("camera_info_url").as_string());

    camera_.setComId(com_.id());
    int camera_number = this->get_parameter("camera_number").as_int();
    try {
        camera_.setCameraNumber(camera_number);
    } catch (const rec::robotino::api2::RobotinoException& e) {
        RCLCPP_ERROR(this->get_logger(), "Cannot select camera %d: %s", camera_number, e.what());
    }

    std::string hostname = this->get_parameter("hostname").as_string();
    com_.setAddress(hostname.c_str());
    com_.connectToServer(true);
    RCLCPP_INFO(this->get_logger(), "Connecting to Robotino with host IP %s\n", hostname.c_str());

    int image_width = this->get_parameter("image_width").as_int();
    int image_height = this->get_parameter("image_height").as_int();
    if (image_width > 0 && image_height > 0) {
        std::string image_format = this->get_parameter("image_format").as_string();
        try {
            camera_.setFormat(image_width, image_height, image_format.c_str());
        } catch (const rec::robotino::api2::RobotinoException& e) {
            RCLCPP_ERROR(this->get_logger(), "Cannot set camera format %dx%d %s: %s", image_width,
                         image_height, image_format.c_str(), e.what());
        }
    }

    // Com::processEvents() dispatches events of all API2 objects associated with com_, including camera_
    auto timer_period = std::chrono::milliseconds(this->get_parameter("timer_period_ms").as_int());
    timer_ = this->create_wall_timer(timer_period, std::bind(&ComROS::processEvents, &com_));
}
