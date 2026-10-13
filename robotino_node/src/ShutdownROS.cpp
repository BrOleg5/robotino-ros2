/*
 * ShutdownROS.cpp
 *
 *  Created on: 13.10.2026
 *      Author: BrOleg5
 */

#include <string>

#include "rec/robotino/api2/RobotinoException.h"

#include "ShutdownROS.h"

using std::placeholders::_1;
using std::placeholders::_2;

ShutdownROS::ShutdownROS(rclcpp::Node* parent_node_ptr) : logger_(parent_node_ptr->get_logger()) {
    std::string node_name = parent_node_ptr->get_name();
    shutdown_server_ = parent_node_ptr->create_service<std_srvs::srv::Trigger>(
        "/" + node_name + "/shutdown", std::bind(&ShutdownROS::shutdownCallback, this, _1, _2));
}

void ShutdownROS::shutdownCallback(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                   std_srvs::srv::Trigger::Response::SharedPtr res) {
    try {
        shutdown();
        res->success = true;
        res->message = "Robotino is shutting down";
        RCLCPP_WARN(logger_, "%s", res->message.c_str());
    } catch (const rec::robotino::api2::RobotinoException& e) {
        res->success = false;
        res->message = e.what();
        RCLCPP_ERROR(logger_, "Failed to shut down Robotino: %s", e.what());
    }
}
