/*
 * ComROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 * 	Edited on: 02.07.2024
 * 		Author: BrOleg5
 */

#include "ComROS.h"

ComROS::ComROS(rclcpp::Node::SharedPtr parent_node_ptr) {
    parent_node_ptr_ = parent_node_ptr;
    this->setName(parent_node_ptr_->get_name());
}

void ComROS::errorEvent(const char* errorString) {
    RCLCPP_ERROR(parent_node_ptr_->get_logger(), errorString);
}

void ComROS::connectedEvent() {
    RCLCPP_INFO(parent_node_ptr_->get_logger(), "Connected to Robotino with address %s", this->address());
}

void ComROS::connectionClosedEvent() {
    RCLCPP_INFO(parent_node_ptr_->get_logger(), "Disconnected from Robotino with address %s", this->address());
}

void ComROS::logEvent(const char* message, int level) {
    RCLCPP_INFO(parent_node_ptr_->get_logger(), "%d: %s", level, message);
}
