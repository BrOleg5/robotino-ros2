/*
 * OmniDriveROS.h
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 09.07.2024
 *      Author: BrOleg5
 */

#ifndef OMNIDRIVEROS_H_
#define OMNIDRIVEROS_H_

#include "rec/robotino/api2/OmniDrive.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class OmniDriveROS : public rec::robotino::api2::OmniDrive {
  public:
    OmniDriveROS(rclcpp::Node* parent_node_ptr);
    ~OmniDriveROS() {}

    void setSpeedLimit(double max_linear_vel, double max_angular_vel);

  private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    double max_linear_vel_;
    double max_angular_vel_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    double limit(double speed, double speed_limit);
};

#endif /* OMNIDRIVEROS_H_ */
