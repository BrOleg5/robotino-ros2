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

#include "nlohmann/json.hpp"

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
    : parent_node_ptr_(parent_node_ptr),
      node_name_(parent_node_ptr->get_name()),
      logger_(parent_node_ptr->get_logger()),
      battery_state_msg_(),
      battery_low_msg_(),
      has_readings_(false),
      festool_charger_(this),
      has_charger_info_(false),
      accus_charging_(false),
      accus_percentage_(NAN) {
    clock_ptr_ = parent_node_ptr->get_clock();
    battery_state_pub_ = parent_node_ptr->create_publisher<BatteryState>("/" + node_name_ + "/battery_state", 10);
    // Published only on change, so latched for late subscribers
    battery_low_pub_ = parent_node_ptr->create_publisher<std_msgs::msg::Bool>("/" + node_name_ + "/battery_low",
                                                                              rclcpp::QoS(1).transient_local());

    // Robotino API2 reports neither capacity nor temperature, charge is known only for Festool batteries
    battery_state_msg_.temperature = NAN;
    battery_state_msg_.charge = NAN;
    battery_state_msg_.capacity = NAN;
    battery_state_msg_.design_capacity = NAN;
    battery_state_msg_.percentage = NAN;
    battery_state_msg_.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    battery_state_msg_.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state_msg_.present = true;
}

void PowerManagementROS::setComId(const rec::robotino::api2::ComId& id) {
    PowerManagement::setComId(id);
    festool_charger_.setComId(id);
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
    if (has_charger_info_) {
        battery_state_msg_.percentage = accus_percentage_;
        if (accus_charging_) {
            battery_state_msg_.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        } else {
            battery_state_msg_.power_supply_status = ext_power ? BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING
                                                               : BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        }
    } else {
        // Without charger info assume that the chargers charge the batteries from the external power supply
        battery_state_msg_.power_supply_status = ext_power ? BatteryState::POWER_SUPPLY_STATUS_CHARGING
                                                           : BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }
    battery_state_pub_->publish(battery_state_msg_);

    if (!has_readings_ || batteryLow != battery_low_msg_.data) {
        if (batteryLow) {
            // The shutdown counter is in ms
            RCLCPP_WARN(logger_, "Battery low (%.1f V), Robotino shuts down in %d s", battery_voltage,
                        batteryLowShutdownCounter / 1000);
        }
        battery_low_msg_.data = batteryLow;
        battery_low_pub_->publish(battery_low_msg_);
    }
    has_readings_ = true;
}

void PowerManagementROS::FestoolChargerReader::festool_charger_infoEvent(const char* data,
                                                                       const unsigned int dataSize) {
    owner_->chargerInfoEvent(std::string(data, dataSize));
}

// The charger info is JSON with arrays indexed by battery slot, the same as http://<robotino>/data/festoolcharger. capacities are in %, negative for an empty slot;
// voltages are in V. accuConnected marks the packs connected to the system power at the moment:
// the charger electronics switch between the packs, so a present pack may be disconnected.
void PowerManagementROS::chargerInfoEvent(const std::string& data) {
    std::vector<int> connected, loading, capacities;
    std::vector<float> voltages;
    try {
        const nlohmann::json info = nlohmann::json::parse(data);
        const auto& payload = info.at("payload");
        payload.at("accuConnected").get_to(connected);
        payload.at("accuLoading").get_to(loading);
        payload.at("capacities").get_to(capacities);
        payload.at("voltages").get_to(voltages);
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_ptr_, 10000, "Failed to parse Festool charger info: %s", e.what());
        return;
    }
    const size_t num_accus = std::min({connected.size(), loading.size(), capacities.size(), voltages.size()});
    if (accus_.size() < num_accus) {
        accus_.resize(num_accus);
    }

    const auto stamp = clock_ptr_->now();
    bool charging = false;
    float percentage_sum = 0.0f;
    int percentage_count = 0;
    for (size_t i = 0; i < num_accus; i++) {
        Accu& accu = accus_[i];
        const bool present = capacities[i] >= 0;
        if (!accu.pub) {
            if (!present) {
                continue;
            }
            accu.pub = parent_node_ptr_->create_publisher<BatteryState>(
                "/" + node_name_ + "/batteries/accu" + std::to_string(i), 10);
            accu.msg.current = NAN;
            accu.msg.charge = NAN;
            accu.msg.capacity = NAN;
            accu.msg.design_capacity = NAN;
            accu.msg.temperature = NAN;
            accu.msg.power_supply_health = BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
            accu.msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
            accu.msg.location = "accu" + std::to_string(i);
            RCLCPP_INFO(logger_, "Battery pack found in slot %zu", i);
        }
        accu.msg.header.stamp = stamp;
        accu.msg.present = present;
        accu.msg.voltage = present ? voltages[i] : NAN;
        accu.msg.percentage = present ? static_cast<float>(capacities[i]) / 100.0f : NAN;
        if (!present) {
            accu.msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        } else if (loading[i] != 0) {
            accu.msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        } else if (connected[i] != 0) {
            accu.msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        } else {
            accu.msg.power_supply_status = BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
        }
        accu.pub->publish(accu.msg);

        charging = charging || (present && loading[i] != 0);
        if (present) {
            percentage_sum += accu.msg.percentage;
            percentage_count++;
        }
    }

    has_charger_info_ = true;
    accus_charging_ = charging;
    // Packs are connected in parallel; without their capacities the mean charge is the best estimate
    accus_percentage_ = percentage_count > 0 ? percentage_sum / static_cast<float>(percentage_count) : NAN;
}
