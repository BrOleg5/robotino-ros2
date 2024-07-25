/*
 * RobotinoCameraNode.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 19.07.2024
 *      Author: BrOleg5
 */

#ifndef ROBOTINOCAMERANODE_H
#define ROBOTINOCAMERANODE_H

#include "rclcpp/rclcpp.hpp"

#include "ComROS.h"
#include "CameraROS.h"

class RobotinoCameraNode : public rclcpp::Node {
  public:
    RobotinoCameraNode();
    ~RobotinoCameraNode() {}

  private:
    rclcpp::TimerBase::SharedPtr timer_;

    ComROS com_;
    CameraROS camera_;
};

#endif /* ROBOTINOCAMERANODE_H */
