/*
 * RTONode.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 09.07.2024
 *      Author: BrOleg5
 */
#include <string>
#include <chrono>

#include "RobotinoNode.h"

RobotinoNode::RobotinoNode()
    : Node("robotino"),
      bumper_(this->make_shared()),
      com_(this->make_shared()),
      distance_sensor_array_(this->make_shared()),
      motor_array_(this->make_shared()),
      omni_drive_(this->make_shared()) {
    this->declare_parameter("hostname", "192.168.1.0");
    this->declare_parameter("max_linear_vel", 1.0);
    this->declare_parameter("max_angular_vel", 3.0);
    this->declare_parameter("timer_period_ms", 50);

    initModules();

    std::string hostname = this->get_parameter("hostname").as_string();
    com_.setAddress(hostname.c_str());
    com_.connectToServer(true);
    RCLCPP_INFO(this->get_logger(), "Connecting to Robotino with host IP %s\n", hostname.c_str());

    auto timer_period = std::chrono::milliseconds(this->get_parameter("timer_period_ms").as_int());
    timer_ = this->create_wall_timer(timer_period, std::bind(&RobotinoNode::timerCallback));
}

void RobotinoNode::initModules() {
    bumper_.setComId(com_.id());
    distance_sensor_array_.setComId(com_.id());
    omni_drive_.setComId(com_.id());
    omni_drive_.setSpeedLimit(this->get_parameter("max_linear_vel").as_double(),
                              this->get_parameter("max_angular_vel").as_double());
}

void RobotinoNode::timerCallback() {
    com_.processEvents();
}
