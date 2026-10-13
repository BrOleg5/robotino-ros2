/*
 * PowerManagementROS.cpp
 *
 *  Created on: 13.10.2026
 *      Author: BrOleg5
 */

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

#include "PowerManagementROS.h"

using BatteryState = sensor_msgs::msg::BatteryState;

// Robotino 4 is supplied by Festool lithium-ion batteries, Robotino 3 by lead-acid or NiMH ones.
// The battery type strings of Robotino API2 are not documented, so they are matched loosely;
// lead-acid has no technology constant in BatteryState and stays unknown.
static uint8_t technologyFromType(std::string type) {
    std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return std::tolower(c); });
    if (type.find("nimh") != std::string::npos) {
        return BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
    }
    if (type.find("lipo") != std::string::npos) {
        return BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
    }
    if (type.find("life") != std::string::npos) {
        return BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
    }
    if (type.find("li") != std::string::npos || type.find("festool") != std::string::npos) {
        return BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    }
    return BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

PowerManagementROS::PowerManagementROS(rclcpp::Node* parent_node_ptr)
    : logger_(parent_node_ptr->get_logger()), battery_state_msg_(), battery_low_msg_(), has_readings_(false) {
    clock_ptr_ = parent_node_ptr->get_clock();
    std::string node_name = parent_node_ptr->get_name();
    battery_state_pub_ =
        parent_node_ptr->create_publisher<BatteryState>("/" + node_name + "/battery_state", 10);
    // Published only on change, so latched for late subscribers
    battery_low_pub_ = parent_node_ptr->create_publisher<std_msgs::msg::Bool>("/" + node_name + "/battery_low",
                                                                              rclcpp::QoS(1).transient_local());

    // Robotino API2 reports neither charge nor capacity
    battery_state_msg_.temperature = NAN;
    battery_state_msg_.charge = NAN;
    battery_state_msg_.capacity = NAN;
    battery_state_msg_.design_capacity = NAN;
    battery_state_msg_.percentage = NAN;
    battery_state_msg_.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    battery_state_msg_.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state_msg_.present = true;
}

void PowerManagementROS::readingsEvent(float battery_voltage, float system_current, bool ext_power,
                                       int /*num_chargers*/, const char* batteryType, bool batteryLow,
                                       int batteryLowShutdownCounter) {
    if (!has_readings_) {
        RCLCPP_INFO(logger_, "Battery type: %s", batteryType);
        battery_state_msg_.power_supply_technology = technologyFromType(batteryType);
    }

    battery_state_msg_.header.stamp = clock_ptr_->now();
    battery_state_msg_.voltage = battery_voltage;
    // Current consumed by the robot, negative as the battery is discharging
    battery_state_msg_.current = -system_current;
    // With the external power supply plugged in the batteries are charged by the chargers
    battery_state_msg_.power_supply_status = ext_power ? BatteryState::POWER_SUPPLY_STATUS_CHARGING
                                                       : BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    battery_state_pub_->publish(battery_state_msg_);

    if (!has_readings_ || batteryLow != battery_low_msg_.data) {
        if (batteryLow) {
            RCLCPP_WARN(logger_, "Battery low (%.1f V), Robotino shuts down in %d s", battery_voltage,
                        batteryLowShutdownCounter);
        }
        battery_low_msg_.data = batteryLow;
        battery_low_pub_->publish(battery_low_msg_);
    }
    has_readings_ = true;
}
