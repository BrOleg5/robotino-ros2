/*
 * main.cpp
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 *  Edited on: 19.07.2024
 *      Author: BrOleg5
 */
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "RobotinoCameraNode.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotinoCameraNode>());
    rclcpp::shutdown();
    return 0;
}
