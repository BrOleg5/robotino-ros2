/*
 * RobotinoCameraNode.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 19.07.2024
 *      Author: BrOleg5
 */

#include "RobotinoCameraNode.h"

RobotinoCameraNode::RobotinoCameraNode() : Node("camera_node"), com_(this), camera_(this) {
    this->declare_parameter("hostname", "192.168.0.1");
    this->declare_parameter("timer_period_ms", 50);
    this->declare_parameter("camera_number", 0);

    std::string hostname = this->get_parameter("hostname").as_string();
    com_.setAddress(hostname.c_str());
    com_.connectToServer(true);
    RCLCPP_INFO(this->get_logger(), "Connecting to Robotino with host IP %s\n", hostname.c_str());

    int camera_number = this->get_parameter("camera_number").as_int();
    camera_.setCameraNumber(camera_number);

    auto timer_period = std::chrono::milliseconds(this->get_parameter("timer_period_ms").as_int());
    timer_ = this->create_wall_timer(timer_period, std::bind(&CameraROS::processEvents, &camera_));
}
