/*
 * LaserRangeFinderROS.h
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 15.07.2024
 *      Author: BrOleg5
 */

#ifndef LASERRANGEFINDERROS_H_
#define LASERRANGEFINDERROS_H_

#include "rec/robotino/api2/LaserRangeFinder.h"
#include "rec/robotino/api2/LaserRangeFinderReadings.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserRangeFinderROS : public rec::robotino::api2::LaserRangeFinder {
  public:
    LaserRangeFinderROS(rclcpp::Node* parent_node_ptr, int number = 0);
    ~LaserRangeFinderROS() {}

  private:
    rclcpp::Clock::SharedPtr clock_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

    sensor_msgs::msg::LaserScan laser_scan_msg_;

    void scanEvent(const rec::robotino::api2::LaserRangeFinderReadings& scan);
};

#endif /* LASERRANGEFINDERROS_H_ */
