/*
 * PowerManagementROS.h
 *
 *  Created on: 13.10.2026
 *      Author: BrOleg5
 */

#ifndef POWERMANAGEMENTROS_H_
#define POWERMANAGEMENTROS_H_

#include "rec/robotino/api2/PowerManagement.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

class PowerManagementROS : public rec::robotino::api2::PowerManagement {
  public:
    PowerManagementROS(rclcpp::Node* parent_node_ptr);
    ~PowerManagementROS() {}

  private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
    // BatteryState has no field for the low battery warning of Robotino, so it is published separately
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_low_pub_;
    sensor_msgs::msg::BatteryState battery_state_msg_;
    std_msgs::msg::Bool battery_low_msg_;
    bool has_readings_;

    void readingsEvent(float battery_voltage, float system_current, bool ext_power, int num_chargers,
                       const char* batteryType, bool batteryLow, int batteryLowShutdownCounter) override;
};

#endif /* POWERMANAGEMENTROS_H_ */
