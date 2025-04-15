/*
 * CameraROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 19.07.2024
 *      Author: BrOleg5
 */

#ifndef CAMERAROS_H_
#define CAMERAROS_H_

#include "rec/robotino/api2/Camera.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraROS : public rec::robotino::api2::Camera {
  public:
    CameraROS(rclcpp::Node* parent_node_ptr);
    ~CameraROS() {}

  private:
    rclcpp::Clock::SharedPtr clock_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr streaming_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr caminfor_pub_;

    sensor_msgs::msg::Image img_msg_;
    sensor_msgs::msg::CameraInfo caminfo_msg_;

    void imageReceivedEvent(const unsigned char* data,
                            unsigned int dataSize,
                            unsigned int width,
                            unsigned int height,
                            unsigned int step);
};

#endif /* CAMERAROS_H_ */
