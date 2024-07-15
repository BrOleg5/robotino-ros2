/*
 * LaserRangeFinderROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 15.07.2024
 *      Author: BrOleg5
 *
 */
#include <string>

#include "LaserRangeFinderROS.h"

LaserRangeFinderROS::LaserRangeFinderROS(rclcpp::Node* parent_node_ptr, int number) : laser_scan_msg_() {
    std::string topic_name;
    if (number == 0)
        topic_name = "laser_scan";
    else
        topic_name = "laser_scan" + std::to_string(number);
    laser_scan_pub_ = parent_node_ptr->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
    setLaserRangeFinderNumber(number);

    clock_ptr_ = parent_node_ptr->get_clock();

    laser_scan_msg_.header.frame_id = "laser_link";
}

void LaserRangeFinderROS::scanEvent(const rec::robotino::api2::LaserRangeFinderReadings& scan) {
    laser_scan_msg_.header.stamp = clock_ptr_->now();

    laser_scan_msg_.angle_min = scan.angle_min;
    laser_scan_msg_.angle_max = scan.angle_max;
    laser_scan_msg_.angle_increment = scan.angle_increment;
    laser_scan_msg_.time_increment = scan.time_increment;
    laser_scan_msg_.scan_time = scan.scan_time;
    laser_scan_msg_.range_min = scan.range_min;
    laser_scan_msg_.range_max = scan.range_max;

    unsigned int numRanges, numIntensities;
    const float* ranges;
    const float* intensities;
    scan.ranges(&ranges, &numRanges);
    scan.intensities(&intensities, &numIntensities);

    laser_scan_msg_.ranges.resize(numRanges);
    laser_scan_msg_.intensities.resize(numIntensities);

    if (ranges != NULL) {
        memcpy(laser_scan_msg_.ranges.data(), ranges, numRanges * sizeof(float));
    }

    if (intensities != NULL) {
        memcpy(laser_scan_msg_.intensities.data(), intensities, numIntensities * sizeof(float));
    }

    // Publish the message
    if (numRanges > 0 || numIntensities > 0)
        laser_scan_pub_->publish(laser_scan_msg_);
}
