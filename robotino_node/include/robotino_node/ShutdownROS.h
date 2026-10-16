/*
 * ShutdownROS.h
 *
 *  Created on: 13.10.2026
 *      Author: BrOleg5
 */

#ifndef SHUTDOWNROS_H_
#define SHUTDOWNROS_H_

#include "rec/robotino/api2/Shutdown.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class ShutdownROS : public rec::robotino::api2::Shutdown {
  public:
    ShutdownROS(rclcpp::Node* parent_node_ptr);
    ~ShutdownROS() {}

  private:
    rclcpp::Logger logger_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shutdown_server_;

    void shutdownCallback(std_srvs::srv::Trigger::Request::SharedPtr req,
                          std_srvs::srv::Trigger::Response::SharedPtr res);
};

#endif /* SHUTDOWNROS_H_ */
