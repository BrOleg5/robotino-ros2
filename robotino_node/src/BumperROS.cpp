/*
 * BumperROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 02.07.2024
 *      Author: BrOleg5
 */

#include <string>

#include "BumperROS.h"

BumperROS::BumperROS(rclcpp::Node::SharedPtr parent_node_ptr) : bumper_msg_() {
    std::string node_name = parent_node_ptr->get_name();
    bumper_pub_ = parent_node_ptr->create_publisher<std_msgs::msg::Bool>("/" + node_name + "/bumper", 10);
}

void BumperROS::bumperEvent(bool hasContact) {
    bumper_msg_.data = hasContact;
    bumper_pub_->publish(bumper_msg_);
}
