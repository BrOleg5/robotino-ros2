/*
 * ComROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 13.10.2026
 *      Author: BrOleg5
 */

#include "ComROS.h"

ComROS::ComROS(rclcpp::Node* parent_node_ptr) : logger_(parent_node_ptr->get_logger()) {
    this->setName(parent_node_ptr->get_name());
}

void ComROS::errorEvent(const char* errorString) {
    RCLCPP_ERROR(logger_, "%s", errorString);
}

void ComROS::connectedEvent() {
    connected_ = true;
    RCLCPP_INFO(logger_, "Connected to Robotino with address %s", this->address());
}

void ComROS::connectionClosedEvent() {
    if (connected_.exchange(false)) {
        RCLCPP_INFO(logger_, "Disconnected from Robotino with address %s", this->address());
    }
}

void ComROS::logEvent(const char* message, int level) {
    // Internal API2 messages, e.g. connection state changes
    RCLCPP_DEBUG(logger_, "%d: %s", level, message);
}
