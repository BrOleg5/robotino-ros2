/*
 * OmniDriveROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 09.07.2024
 *      Author: BrOleg5
 */
#include <cmath>

#include "OmniDriveROS.h"
#include "misc.hpp"

using std::placeholders::_1;

OmniDriveROS::OmniDriveROS(rclcpp::Node::SharedPtr parent_node_ptr) : max_linear_vel_(1.0), max_angular_vel_(4) {
    cmd_vel_sub_ = parent_node_ptr->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&OmniDriveROS::cmdVelCallback, this, _1));
}

double OmniDriveROS::limit(double speed, double speed_limit) {
    if (std::abs(speed) <= speed_limit) {
        return speed;
    } else {
        return speed_limit * sgn(speed);
    }
}

void OmniDriveROS::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double linear_x = limit(msg->linear.x, max_linear_vel_);
    double linear_y = limit(msg->linear.y, max_linear_vel_);
    double angular = limit(msg->angular.z, max_angular_vel_);
    setVelocity(linear_x, linear_y, angular);
}

void OmniDriveROS::setSpeedLimit(double max_linear_vel, double max_angular_vel) {
    max_linear_vel_ = max_linear_vel;
    max_angular_vel_ = max_angular_vel;
}
