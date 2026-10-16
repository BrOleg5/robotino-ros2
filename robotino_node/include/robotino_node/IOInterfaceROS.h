/*
 * IOInterfaceROS.h
 *
 *  Created on: 13.10.2026
 *      Author: BrOleg5
 */

#ifndef IOINTERFACEROS_H_
#define IOINTERFACEROS_H_

#include <string>
#include <vector>

#include "rec/robotino/api2/AnalogInputArray.h"
#include "rec/robotino/api2/DigitalInputArray.h"
#include "rec/robotino/api2/DigitalOutputArray.h"
#include "rec/robotino/api2/RelayArray.h"

#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"

// Robotino I/O interface: digital and analog inputs, digital outputs and relays.
// States of all channels are published in one message, the same way as gpio_controllers/GpioCommandController does.
class IOInterfaceROS {
  public:
    IOInterfaceROS(rclcpp::Node* parent_node_ptr);
    ~IOInterfaceROS() {}

    void setComId(const rec::robotino::api2::ComId& id);
    // Publishes the states if any of them changed since the last call
    void publishState();

  private:
    using GpioMsg = control_msgs::msg::DynamicInterfaceGroupValues;

    // Indices of the groups in gpio_state_msg_
    enum Group : size_t { DIGITAL_INPUTS, ANALOG_INPUTS, DIGITAL_OUTPUTS, RELAYS };

    class AnalogInputs : public rec::robotino::api2::AnalogInputArray {
      public:
        AnalogInputs(IOInterfaceROS* owner) : owner_(owner) {}

      private:
        IOInterfaceROS* owner_;
        void valuesChangedEvent(const float* values, unsigned int size) override;
    };

    class DigitalInputs : public rec::robotino::api2::DigitalInputArray {
      public:
        DigitalInputs(IOInterfaceROS* owner) : owner_(owner) {}

      private:
        IOInterfaceROS* owner_;
        void valuesChangedEvent(const int* values, unsigned int size) override;
    };

    class DigitalOutputs : public rec::robotino::api2::DigitalOutputArray {
      public:
        DigitalOutputs(IOInterfaceROS* owner) : owner_(owner) {}

      private:
        IOInterfaceROS* owner_;
        void digitalOutputStatusChangedEvent(const int* values, unsigned int size) override;
    };

    class Relays : public rec::robotino::api2::RelayArray {
      public:
        Relays(IOInterfaceROS* owner) : owner_(owner) {}

      private:
        IOInterfaceROS* owner_;
        void relayStatusChangedEvent(const int* values, unsigned int size) override;
    };

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_ptr_;
    rclcpp::Publisher<GpioMsg>::SharedPtr gpio_state_pub_;
    rclcpp::Subscription<GpioMsg>::SharedPtr gpio_command_sub_;
    GpioMsg gpio_state_msg_;
    bool state_changed_;

    // Last commanded values, channels missing in a command keep them
    std::vector<int> digital_output_commands_;
    std::vector<int> relay_commands_;

    AnalogInputs analog_inputs_;
    DigitalInputs digital_inputs_;
    DigitalOutputs digital_outputs_;
    Relays relays_;

    void addGroup(const std::string& group_name, const std::string& channel_prefix, unsigned int num_channels);
    template <typename T>
    void updateState(Group group, const T* values, unsigned int size);

    void gpioCommandCallback(const GpioMsg::SharedPtr msg);
    // Returns true if at least one command was applied
    bool applyCommands(Group group, const control_msgs::msg::InterfaceValue& group_values,
                       std::vector<int>& commands);
};

#endif /* IOINTERFACEROS_H_ */
