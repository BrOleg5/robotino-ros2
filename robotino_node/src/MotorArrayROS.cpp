/*
 * MotorArrayROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 * 	Edited on: 03.07.2024
 * 		Author: BrOleg5
 */

#include <string>

#include "MotorArrayROS.h"

MotorArrayROS::MotorArrayROS(rclcpp::Node::SharedPtr parent_node_ptr) : joint_state_msg_() {
    parent_node_ptr_ = parent_node_ptr;
    std::string node_name = parent_node_ptr_->get_name();
    joint_state_pub_ =
        parent_node_ptr_->create_publisher<sensor_msgs::msg::JointState>("/" + node_name + "/joint_state", 10);

    joint_state_msg_.header.frame_id = "base_link";
    joint_state_msg_.name.resize(3);
    joint_state_msg_.position.resize(3, 0.0);
    joint_state_msg_.velocity.resize(3, 0.0);
    for (size_t i = 0; i < joint_state_msg_.name.size(); i++) {
        joint_state_msg_.name[i] = "wheel" + std::to_string(i) + "_link";
    }
    hasPositions = false;
    hasVelocities = false;
}

void MotorArrayROS::velocitiesChangedEvent(const float* velocities, unsigned int size) {
    for (size_t i = 0; i < size; i++) {
        joint_state_msg_.velocity[i] = static_cast<double>(velocities[i]);
    }
    hasVelocities = true;
    if (hasPositions && hasVelocities) {
        publish();
    }
}

void MotorArrayROS::positionsChangedEvent(const float* positions, unsigned int size) {
    for (size_t i = 0; i < size; i++) {
        joint_state_msg_.position[i] = static_cast<double>(positions[i]);
    }
    hasPositions = true;
    if (hasPositions && hasVelocities) {
        publish();
    }
}
