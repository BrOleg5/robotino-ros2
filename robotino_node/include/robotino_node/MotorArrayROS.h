/*
 * MotorArrayROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 * 	Edited on: 03.07.2024
 * 		Author: BrOleg5
 */

#ifndef MOTORARRAYROS_H_
#define MOTORARRAYROS_H_

#include "rec/robotino/api2/MotorArray.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MotorArrayROS : public rec::robotino::api2::MotorArray {
  public:
    MotorArrayROS(rclcpp::Node::SharedPtr parent_node_ptr);
    ~MotorArrayROS() {}

  private:
    rclcpp::Node::SharedPtr parent_node_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    sensor_msgs::msg::JointState joint_state_msg_;
    bool hasPositions;
    bool hasVelocities;

    void velocitiesChangedEvent(const float* velocities, unsigned int size);
    void positionsChangedEvent(const float* positions, unsigned int size);

    inline void publish() {
        joint_state_msg_.header.stamp = parent_node_ptr_->get_clock()->now();
        joint_state_pub_->publish(joint_state_msg_);
        hasPositions = false;
        hasVelocities = false;
    }
};
#endif /* MOTORARRAYROS_H_ */
