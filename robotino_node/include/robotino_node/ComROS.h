/*
 * ComROS.h
 *
 *  Created on: 06.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 13.10.2026
 *      Author: BrOleg5
 */

#ifndef COMROS_H_
#define COMROS_H_

#include "rec/robotino/api2/Com.h"

#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <string>

class ComROS : public rec::robotino::api2::Com {
  public:
    ComROS(rclcpp::Node* parent_node_ptr);
    ~ComROS() {}

  private:
    rclcpp::Logger logger_;
    // Connection state for logging only, connection attempts that fail are not reported as disconnections
    std::atomic<bool> connected_ = false;
    void errorEvent(const char* errorString);
    void connectedEvent();
    void connectionClosedEvent();
    void logEvent(const char* message, int level);
};

#endif /* COMROS_H_ */
