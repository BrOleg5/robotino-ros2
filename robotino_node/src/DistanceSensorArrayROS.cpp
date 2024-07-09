/*
 * DistanceSensorArrayROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 03.07.2024
 * 	    Author: BrOleg5
 */

#include "DistanceSensorArrayROS.h"
#include <string>
#include <cmath>

DistanceSensorArrayROS::DistanceSensorArrayROS(rclcpp::Node* parent_node_ptr) : range_msg_() {
    clock_ptr_ = parent_node_ptr->get_clock();
    std::string node_name = parent_node_ptr->get_name();
    for (unsigned int i = 0; i < distance_pubs_.size(); i++) {
        distance_pubs_[i] =
            parent_node_ptr->create_publisher<sensor_msgs::msg::Range>("/" + node_name + "/ir" + std::to_string(i), 10);
    }

    range_msg_.radiation_type = range_msg_.INFRARED;
    range_msg_.field_of_view = 0.7f;  // Will correct in the future
    range_msg_.min_range = 0.04f;
    range_msg_.max_range = 0.30f;
}

void DistanceSensorArrayROS::distancesChangedEvent(const float* distances, unsigned int size) {
    range_msg_.header.stamp = clock_ptr_->now();

    for (unsigned int i = 0; i < size; ++i) {
        if ((distances[i] >= range_msg_.min_range) && (distances[i] <= range_msg_.max_range)) {
            range_msg_.header.frame_id = distance_pubs_[i]->get_topic_name();
            range_msg_.range = distances[i];
            distance_pubs_[i]->publish(range_msg_);
        }
    }
}
