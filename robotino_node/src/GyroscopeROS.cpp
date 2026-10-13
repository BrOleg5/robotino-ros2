/*
 * GyroscopeROS.cpp
 *
 *  Created on: 13.10.2026
 *      Author: BrOleg5
 */

#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "GyroscopeROS.h"

GyroscopeROS::GyroscopeROS(rclcpp::Node* parent_node_ptr) : imu_msg_() {
    clock_ptr_ = parent_node_ptr->get_clock();
    std::string node_name = parent_node_ptr->get_name();
    imu_pub_ = parent_node_ptr->create_publisher<sensor_msgs::msg::Imu>("/" + node_name + "/imu", 10);

    imu_msg_.header.frame_id = "base_link";
    // -1 in the first element marks the data as not provided
    imu_msg_.linear_acceleration_covariance[0] = -1.0;
}

void GyroscopeROS::gyroscopeEvent(float angle, float rate) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle);

    imu_msg_.header.stamp = clock_ptr_->now();
    imu_msg_.orientation = tf2::toMsg(q);
    imu_msg_.angular_velocity.z = rate;
    imu_pub_->publish(imu_msg_);
}
