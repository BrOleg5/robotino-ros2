/*
 * MotorArrayROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 03.07.2024
 * 	    Author: BrOleg5
 */

#include <string>

#include "MotorArrayROS.h"
#include "misc.hpp"

MotorArrayROS::MotorArrayROS(rclcpp::Node* parent_node_ptr) : joint_state_msg_(), gearRatio(32.0) {
    clock_ptr_ = parent_node_ptr->get_clock();
    std::string node_name = parent_node_ptr->get_name();
    joint_state_pub_ =
        parent_node_ptr->create_publisher<sensor_msgs::msg::JointState>("/" + node_name + "/joint_states", 10);

    joint_state_msg_.header.frame_id = "base_link";
    joint_state_msg_.name.resize(3);
    joint_state_msg_.position.resize(3, 0.0);
    joint_state_msg_.velocity.resize(3, 0.0);
    for (size_t i = 0; i < joint_state_msg_.name.size(); i++) {
        joint_state_msg_.name[i] = "wheel" + std::to_string(i) + "_joint";
    }
    hasPositions = false;
    hasVelocities = false;

    // software encoder resolution = hardware encoder resolution * (front edge + back edge)  * number of channels =
    // = 500 * 2 * 2 = 2000
    encoderResolutionPPR = 2000.0;
}

void MotorArrayROS::velocitiesChangedEvent(const float* velocities, unsigned int size) {
    for (size_t i = 0; i < size; i++) {
        // convertion from rpm to rad/s
        joint_state_msg_.velocity[i] = 2.0 * PI<double> * static_cast<double>(velocities[i]) / 60.0 / gearRatio;
    }
    hasVelocities = true;
    if (hasPositions && hasVelocities) {
        publish();
    }
}

void MotorArrayROS::positionsChangedEvent(const int* positions, unsigned int size) {
    for (size_t i = 0; i < size; i++) {
        // conversion from encoders pulses to rad
        joint_state_msg_.position[i] =
            2.0 * PI<double> * static_cast<double>(positions[i]) / encoderResolutionPPR / gearRatio;
    }
    hasPositions = true;
    if (hasPositions && hasVelocities) {
        publish();
    }
}
