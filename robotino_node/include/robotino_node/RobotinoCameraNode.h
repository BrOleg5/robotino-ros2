/*
 * RobotinoCameraNode.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 13.10.2026
 *      Author: BrOleg5
 */

#ifndef ROBOTINOCAMERANODE_H
#define ROBOTINOCAMERANODE_H

#include "rec/robotino/api2/RobotinoException.h"

#include "rclcpp/rclcpp.hpp"

#include "ComROS.h"
#include "CameraROS.h"

class RobotinoCameraNode : public rclcpp::Node {
  public:
    RobotinoCameraNode();
    ~RobotinoCameraNode() {}

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    bool connected_ = false;

    ComROS com_;
    CameraROS camera_;

    void timerCallback();
    // Camera settings are applied on every (re)connection to Robotino
    void applyCameraSettings();
};

#endif /* ROBOTINOCAMERANODE_H */
