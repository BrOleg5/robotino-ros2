/*
 * OdometryROS.cpp
 *
 *  Created on: 07.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#include "OdometryROS.h"
#include "tf2/transform_datatypes.h"
#include "tf2/buffer_core.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/quaternion.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

OdometryROS::OdometryROS(rclcpp::Node::SharedPtr parent_node_ptr) : odometry_transform_broadcaster_(parent_node_ptr) {
    clock_ptr_ = parent_node_ptr->get_clock();
    odometry_pub_ = parent_node_ptr->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    reset_odometry_server_ = parent_node_ptr->create_service<rto_msgs::srv::ResetOdometry>(
        "reset_odometry", std::bind(&OdometryROS::resetOdometryCallback, this, _1, _2));

    if (set(0.0, 0.0, 0.0, 500)) {
        RCLCPP_INFO(parent_node_ptr->get_logger(), "Odometry is reset.")
    } else {
        RCLCPP_WARN(parent_node_ptr->get_logger(), "Odometry is not reset.")
    }

    initMessage();
    initTransform();
}

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

void OdometryROS::initMessage() {
    odometry_msg_.header.frame_id = "odom";
    odometry_msg_.child_frame_id = "base_link";
    odometry_msg_.pose.pose.position.x = 0.0;
    odometry_msg_.pose.pose.position.y = 0.0;
    odometry_msg_.pose.pose.position.z = 0.0;
    odometry_msg_.pose.pose.orientation = createQuaternionMsgFromYaw(0.0);
    odometry_msg_.twist.twist.linear.x = 0.0;
    odometry_msg_.twist.twist.linear.y = 0.0;
    odometry_msg_.twist.twist.linear.z = 0.0;
    odometry_msg_.twist.twist.angular.x = 0.0;
    odometry_msg_.twist.twist.angular.y = 0.0;
    odometry_msg_.twist.twist.angular.z = 0.0;
}

void OdometryROS::initTransform() {
    odometry_transform_.header.frame_id = "odom";
    odometry_transform_.child_frame_id = "base_link";
    odometry_transform_.transform.translation.x = 0.0;
    odometry_transform_.transform.translation.y = 0.0;
    odometry_transform_.transform.translation.z = 0.0;
    odometry_transform_.transform.rotation = createQuaternionMsgFromYaw(0.0);
}

void OdometryROS::readingsEvent(double x,
                                double y,
                                double phi,
                                float vx,
                                float vy,
                                float omega,
                                unsigned int sequence) {
    geometry_msgs::msg::Quaternion orient_quat = createQuaternionMsgFromYaw(phi);

    odometry_msg_.header.stamp = clock_ptr_->now();
    odometry_msg_.pose.pose.position.x = x;
    odometry_msg_.pose.pose.position.y = y;
    odometry_msg_.pose.pose.orientation = orient_quat;
    odometry_msg_.twist.twist.linear.x = vx;
    odometry_msg_.twist.twist.linear.y = vy;
    odometry_msg_.twist.twist.angular.z = omega;

    odometry_transform_.header.stamp = odometry_msg_.header.stamp;
    odometry_transform_.transform.translation.x = x;
    odometry_transform_.transform.translation.y = y;
    odometry_transform_.transform.rotation = orient_quat;

    odometry_pub_->publish(odometry_msg_);
    odometry_transform_broadcaster_.sendTransform(odometry_transform_);
}

void OdometryROS::resetOdometryCallback(rto_msgs::srv::ResetOdometry::Request::SharedPtr req,
                                        rto_msgs::srv::ResetOdometry::Response::SharedPtr res) {
    res->status = set(req->x, req->y, req->phi, 500);
}
