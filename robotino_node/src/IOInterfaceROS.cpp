/*
 * IOInterfaceROS.cpp
 *
 *  Created on: 13.10.2026
 *      Author: BrOleg5
 */

#include <algorithm>
#include <string>

#include "rec/robotino/api2/RobotinoException.h"

#include "IOInterfaceROS.h"

using std::placeholders::_1;

IOInterfaceROS::IOInterfaceROS(rclcpp::Node* parent_node_ptr)
    : logger_(parent_node_ptr->get_logger()),
      gpio_state_msg_(),
      state_changed_(false),
      digital_output_commands_(DigitalOutputs::numDigitalOutputs(), 0),
      relay_commands_(Relays::numRelays(), 0),
      analog_inputs_(this),
      digital_inputs_(this),
      digital_outputs_(this),
      relays_(this) {
    clock_ptr_ = parent_node_ptr->get_clock();
    std::string node_name = parent_node_ptr->get_name();
    gpio_state_pub_ = parent_node_ptr->create_publisher<GpioMsg>("/" + node_name + "/gpio_states", 10);
    gpio_command_sub_ = parent_node_ptr->create_subscription<GpioMsg>(
        "/" + node_name + "/gpio_commands", 10, std::bind(&IOInterfaceROS::gpioCommandCallback, this, _1));

    // Order must match the Group enum. Channel names are the labels on the I/O interface connector.
    addGroup("digital_inputs", "DI", DigitalInputs::numDigitalInputs());
    addGroup("analog_inputs", "AI", AnalogInputs::numAnalogInputs());
    addGroup("digital_outputs", "DQ", DigitalOutputs::numDigitalOutputs());
    addGroup("relays", "R", Relays::numRelays());
}

void IOInterfaceROS::addGroup(const std::string& group_name, const std::string& channel_prefix,
                              unsigned int num_channels) {
    control_msgs::msg::InterfaceValue group_values;
    for (unsigned int i = 0; i < num_channels; i++) {
        group_values.interface_names.push_back(channel_prefix + std::to_string(i + 1));
    }
    group_values.values.resize(num_channels, 0.0);
    gpio_state_msg_.interface_groups.push_back(group_name);
    gpio_state_msg_.interface_values.push_back(group_values);
}

void IOInterfaceROS::setComId(const rec::robotino::api2::ComId& id) {
    analog_inputs_.setComId(id);
    digital_inputs_.setComId(id);
    digital_outputs_.setComId(id);
    relays_.setComId(id);
}

void IOInterfaceROS::publishState() {
    if (!state_changed_) {
        return;
    }
    gpio_state_msg_.header.stamp = clock_ptr_->now();
    gpio_state_pub_->publish(gpio_state_msg_);
    state_changed_ = false;
}

template <typename T>
void IOInterfaceROS::updateState(Group group, const T* values, unsigned int size) {
    auto& state = gpio_state_msg_.interface_values[group].values;
    for (size_t i = 0; i < std::min<size_t>(size, state.size()); i++) {
        state[i] = static_cast<double>(values[i]);
    }
    state_changed_ = true;
}

void IOInterfaceROS::AnalogInputs::valuesChangedEvent(const float* values, unsigned int size) {
    owner_->updateState(ANALOG_INPUTS, values, size);
}

void IOInterfaceROS::DigitalInputs::valuesChangedEvent(const int* values, unsigned int size) {
    owner_->updateState(DIGITAL_INPUTS, values, size);
}

void IOInterfaceROS::DigitalOutputs::digitalOutputStatusChangedEvent(const int* values, unsigned int size) {
    owner_->updateState(DIGITAL_OUTPUTS, values, size);
}

void IOInterfaceROS::Relays::relayStatusChangedEvent(const int* values, unsigned int size) {
    owner_->updateState(RELAYS, values, size);
}

void IOInterfaceROS::gpioCommandCallback(const GpioMsg::SharedPtr msg) {
    if (msg->interface_groups.size() != msg->interface_values.size()) {
        RCLCPP_WARN(logger_, "GPIO command ignored: interface_groups and interface_values sizes differ");
        return;
    }
    for (size_t i = 0; i < msg->interface_groups.size(); i++) {
        const std::string& group_name = msg->interface_groups[i];
        const auto& group_values = msg->interface_values[i];
        try {
            if (group_name == gpio_state_msg_.interface_groups[DIGITAL_OUTPUTS]) {
                if (applyCommands(DIGITAL_OUTPUTS, group_values, digital_output_commands_)) {
                    digital_outputs_.setValues(digital_output_commands_.data(), digital_output_commands_.size());
                }
            } else if (group_name == gpio_state_msg_.interface_groups[RELAYS]) {
                if (applyCommands(RELAYS, group_values, relay_commands_)) {
                    relays_.setValues(relay_commands_.data(), relay_commands_.size());
                }
            } else {
                RCLCPP_WARN(logger_, "GPIO command for unknown or read-only group '%s' ignored", group_name.c_str());
            }
        } catch (const rec::robotino::api2::RobotinoException& e) {
            RCLCPP_ERROR(logger_, "Failed to set group '%s': %s", group_name.c_str(), e.what());
        }
    }
}

bool IOInterfaceROS::applyCommands(Group group, const control_msgs::msg::InterfaceValue& group_values,
                                   std::vector<int>& commands) {
    const std::string& group_name = gpio_state_msg_.interface_groups[group];
    if (group_values.interface_names.size() != group_values.values.size()) {
        RCLCPP_WARN(logger_, "GPIO command for group '%s' ignored: interface_names and values sizes differ",
                    group_name.c_str());
        return false;
    }
    const auto& channel_names = gpio_state_msg_.interface_values[group].interface_names;
    bool applied = false;
    for (size_t i = 0; i < group_values.interface_names.size(); i++) {
        const std::string& name = group_values.interface_names[i];
        auto it = std::find(channel_names.begin(), channel_names.end(), name);
        if (it == channel_names.end()) {
            RCLCPP_WARN(logger_, "GPIO command for unknown channel '%s' in group '%s' ignored", name.c_str(),
                        group_name.c_str());
            continue;
        }
        commands[std::distance(channel_names.begin(), it)] = group_values.values[i] != 0.0 ? 1 : 0;
        applied = true;
    }
    return applied;
}
