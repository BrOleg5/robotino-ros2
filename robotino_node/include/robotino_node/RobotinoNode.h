/*
 * RTONode.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 09.07.2024
 *      Author: BrOleg5
 */

#ifndef ROBOTINONODE_H_
#define ROBOTINONODE_H_

#include "BumperROS.h"
#include "ComROS.h"
#include "DistanceSensorArrayROS.h"
#include "MotorArrayROS.h"
#include "OmniDriveROS.h"

#include "rclcpp/rclcpp.hpp"

class RobotinoNode : public rclcpp::Node {
  public:
    RobotinoNode();
    ~RobotinoNode() {}

  private:
    rclcpp::TimerBase::SharedPtr timer_;

    BumperROS bumper_;
    ComROS com_;
    DistanceSensorArrayROS distance_sensor_array_;
    MotorArrayROS motor_array_;
    OmniDriveROS omni_drive_;

    void initModules();
    void timerCallback();
};

#endif /* ROBOTINONODE_H_ */
