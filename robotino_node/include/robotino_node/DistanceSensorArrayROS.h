/*
 * DistanceSensorArrayROS.h
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 03.07.2024
 *      Author: BrOleg5
 */

#ifndef DISTANCESENSORARRAYROS_H_
#define DISTANCESENSORARRAYROS_H_

#include <array>

#include "rec/robotino/api2/DistanceSensorArray.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class DistanceSensorArrayROS : public rec::robotino::api2::DistanceSensorArray {
  public:
    DistanceSensorArrayROS(rclcpp::Node* parent_node_ptr);
    ~DistanceSensorArrayROS() {}

  private:
    rclcpp::Clock::SharedPtr clock_ptr_;
    std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, 9> distance_pubs_;
    sensor_msgs::msg::Range range_msg_;

    void distancesChangedEvent(const float* distances, unsigned int size);
};

#endif /* DISTANCESENSORARRAYROS_H_ */
