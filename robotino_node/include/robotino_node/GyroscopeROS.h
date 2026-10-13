/*
 * GyroscopeROS.h
 *
 *  Created on: 13.10.2026
 *      Author: BrOleg5
 */

#ifndef GYROSCOPEROS_H_
#define GYROSCOPEROS_H_

#include "rec/robotino/api2/Gyroscope.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Robotino gyroscope measures rotation around the vertical axis only,
// so the message contains yaw and yaw rate, and no linear acceleration.
class GyroscopeROS : public rec::robotino::api2::Gyroscope {
  public:
    GyroscopeROS(rclcpp::Node* parent_node_ptr);
    ~GyroscopeROS() {}

  private:
    rclcpp::Clock::SharedPtr clock_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    sensor_msgs::msg::Imu imu_msg_;

    void gyroscopeEvent(float angle, float rate) override;
};

#endif /* GYROSCOPEROS_H_ */
