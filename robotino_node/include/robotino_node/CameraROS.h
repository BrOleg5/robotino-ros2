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

#include <turbojpeg.h>

#include "rec/robotino/api2/Camera.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/header.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

// Publishes image_raw, image_raw/compressed (image_transport "compressed" transport) and camera_info.
// JPEG images received from Robotino are published to image_raw/compressed as is, without re-encoding,
// and decoded only if image_raw has subscribers. Raw images are JPEG-encoded only if
// image_raw/compressed has subscribers.
class CameraROS : public rec::robotino::api2::Camera {
  public:
    CameraROS(rclcpp::Node* parent_node_ptr);
    ~CameraROS();

    // frame_id: frame of published messages (optical frame: z forward, x right, y down).
    // camera_name, camera_info_url: calibration lookup by camera_info_manager,
    // an empty url means file://${ROS_HOME}/camera_info/<camera_name>.yaml.
    // jpeg_quality: quality of JPEG encoding, used only if Robotino transfers raw images.
    void configure(const std::string& frame_id,
                   const std::string& camera_name,
                   const std::string& camera_info_url,
                   int jpeg_quality);

  private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_pub_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> caminfo_manager_;

    std::string frame_id_;
    int jpeg_quality_ = 80;

    tjhandle decompressor_;
    tjhandle compressor_;
    // Reused JPEG encoding buffer
    unsigned char* jpeg_buf_ = nullptr;
    unsigned long jpeg_buf_size_ = 0;

    void imageReceivedEvent(const unsigned char* data,
                            unsigned int dataSize,
                            unsigned int width,
                            unsigned int height,
                            unsigned int step);
    void settingsChangedEvent(unsigned int width, unsigned int height, const char* format);

    // Return false if the image is dropped
    bool processJpegImage(const std_msgs::msg::Header& header,
                          const unsigned char* data,
                          unsigned int dataSize,
                          int& width,
                          int& height);
    bool processRawImage(const std_msgs::msg::Header& header,
                         const unsigned char* data,
                         unsigned int dataSize,
                         unsigned int width,
                         unsigned int height,
                         unsigned int step);
    void publishCameraInfo(const std_msgs::msg::Header& header, unsigned int width, unsigned int height);
};

#endif /* CAMERAROS_H_ */
