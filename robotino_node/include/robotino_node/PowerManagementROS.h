/*
 * PowerManagementROS.h
 *
 *  Created on: 13.10.2026
 *      Author: BrOleg5
 */

#ifndef POWERMANAGEMENTROS_H_
#define POWERMANAGEMENTROS_H_

#include <string>
#include <vector>

#include "rec/robotino/api2/FestoolCharger.h"
#include "rec/robotino/api2/PowerManagement.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

// Battery voltage and current come from the power management. Robotino 4 also reports charge and state
// of each Festool battery pack through its charger electronics; they are published per pack and
// summarized in the overall battery state.
class PowerManagementROS : public rec::robotino::api2::PowerManagement {
  public:
    PowerManagementROS(rclcpp::Node* parent_node_ptr);
    ~PowerManagementROS() {}

    void setComId(const rec::robotino::api2::ComId& id);

  private:
    using BatteryState = sensor_msgs::msg::BatteryState;

    class FestoolChargerReader : public rec::robotino::api2::FestoolCharger {
      public:
        FestoolChargerReader(PowerManagementROS* owner) : owner_(owner) {}

      private:
        PowerManagementROS* owner_;
        void festool_charger_infoEvent(const char* data, const unsigned int dataSize) override;
    };

    struct Accu {
        rclcpp::Publisher<BatteryState>::SharedPtr pub;
        BatteryState msg;
    };

    rclcpp::Node* parent_node_ptr_;
    std::string node_name_;
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
    // BatteryState has no field for the low battery warning of Robotino, so it is published separately
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_low_pub_;
    sensor_msgs::msg::BatteryState battery_state_msg_;
    std_msgs::msg::Bool battery_low_msg_;
    bool has_readings_;

    FestoolChargerReader festool_charger_;
    bool has_charger_info_;
    // Summary of the battery packs from the last charger info
    bool accus_charging_;
    float accus_percentage_;
    // Battery packs by slot, publishers are created once a pack is connected to the slot
    std::vector<Accu> accus_;

    void chargerInfoEvent(const std::string& data);

    void readingsEvent(float battery_voltage, float system_current, bool ext_power, int num_chargers,
                       const char* batteryType, bool batteryLow, int batteryLowShutdownCounter) override;
};

#endif /* POWERMANAGEMENTROS_H_ */
