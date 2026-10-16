/*
 * MotorArrayROS.cpp
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 03.07.2024
 * 	    Author: BrOleg5
 */

#include <algorithm>
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
    joint_state_msg_.effort.resize(3, 0.0);
    for (size_t i = 0; i < joint_state_msg_.name.size(); i++) {
        joint_state_msg_.name[i] = "wheel" + std::to_string(i) + "_joint";
    }
    hasPositions = false;
    hasVelocities = false;

    // software encoder resolution = hardware encoder resolution * (front edge + back edge)  * number of channels =
    // = 500 * 2 * 2 = 2000
    encoderResolutionPPR = 2000.0;

    // Torque constant of the 24 V Dunkermotoren GR 42x40 from its datasheet:
    // starting torque / (starting current - no load current) = 0.33 N*m / (5.68 A - 0.18 A) = 0.060 N*m/A,
    // continuous rated torque / (continuous current - no load current) = 0.057 N*m / (1.2 A - 0.18 A) = 0.056 N*m/A
    motorTorqueConstant = 0.058;
}

// Positive motor speed of Robotino API2 drives the robot anti-clockwise around its center,
// while positive rotation of wheel joints in robotino_description (and ros2_control
// omni_wheel_drive_controller) drives it clockwise, so the sign is inverted.
void MotorArrayROS::velocitiesChangedEvent(const float* velocities, unsigned int size) {
    // Robotino API2 reports all motor channels of the controller (4), but only the first 3 drive wheels
    for (size_t i = 0; i < std::min<size_t>(size, joint_state_msg_.velocity.size()); i++) {
        // convertion from rpm to rad/s
        joint_state_msg_.velocity[i] = -2.0 * PI<double> * static_cast<double>(velocities[i]) / 60.0 / gearRatio;
    }
    hasVelocities = true;
    if (hasPositions && hasVelocities) {
        publish();
    }
}

void MotorArrayROS::positionsChangedEvent(const int* positions, unsigned int size) {
    for (size_t i = 0; i < std::min<size_t>(size, joint_state_msg_.position.size()); i++) {
        // conversion from encoders pulses to rad
        joint_state_msg_.position[i] =
            -2.0 * PI<double> * static_cast<double>(positions[i]) / encoderResolutionPPR / gearRatio;
    }
    hasPositions = true;
    if (hasPositions && hasVelocities) {
        publish();
    }
}

// Effort is the wheel torque estimated from the motor current, gearbox losses are neglected.
// The last received values go out with positions and velocities.
void MotorArrayROS::currentsChangedEvent(const float* currents, unsigned int size) {
    for (size_t i = 0; i < std::min<size_t>(size, joint_state_msg_.effort.size()); i++) {
        // conversion from A to N*m
        joint_state_msg_.effort[i] = -motorTorqueConstant * gearRatio * static_cast<double>(currents[i]);
    }
}
