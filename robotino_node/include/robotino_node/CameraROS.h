/*
 * CameraROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 13.10.2026
 *      Author: BrOleg5
 */

#ifndef CAMERAROS_H_
#define CAMERAROS_H_

#include <memory>
#include <string>

#include "rec/robotino/api2/Camera.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "image_transport/camera_publisher.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

class CameraROS : public rec::robotino::api2::Camera {
  public:
    CameraROS(rclcpp::Node* parent_node_ptr);
    ~CameraROS() {}

    // frame_id: frame of published messages (optical frame: z forward, x right, y down).
    // camera_name, camera_info_url: calibration lookup by camera_info_manager,
    // an empty url means file://${ROS_HOME}/camera_info/<camera_name>.yaml.
    void configure(const std::string& frame_id,
                   const std::string& camera_name,
                   const std::string& camera_info_url);

  private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_ptr_;
    // Publishes image_raw with all available transports (compressed, zstd, ...) and camera_info
    image_transport::CameraPublisher camera_pub_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> caminfo_manager_;

    sensor_msgs::msg::Image img_msg_;

    void imageReceivedEvent(const unsigned char* data,
                            unsigned int dataSize,
                            unsigned int width,
                            unsigned int height,
                            unsigned int step);
    void settingsChangedEvent(unsigned int width, unsigned int height, const char* format);
};

#endif /* CAMERAROS_H_ */
