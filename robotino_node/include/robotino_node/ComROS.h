/*
 * ComROS.h
 *
 *  Created on: 06.12.2011
 *      Author: indorewala@servicerobotics.eu
 * 	Edited on: 02.07.2024
 * 		Author: BrOleg5
 */

#ifndef COMROS_H_
#define COMROS_H_

#include "rec/robotino/api2/Com.h"

#include "rclcpp/rclcpp.hpp"
#include <string>

class ComROS : public rec::robotino::api2::Com {
  public:
    ComROS(rclcpp::Node::SharedPtr parent_node_ptr);
    ~ComROS() {}

  private:
    rclcpp::Node::SharedPtr parent_node_ptr_;
    void errorEvent(const char* errorString);
    void connectedEvent();
    void connectionClosedEvent();
    void logEvent(const char* message, int level);
};

#endif /* COMROS_H_ */
