#ifndef LEAPMOTION_COMPONENT_HPP_
#define LEAPMOTION_COMPONENT_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <cstring>

#include "Leap.h"
#include "sample_listener.hpp"

using namespace std::chrono_literals;
using namespace Leap;

class LeapMotion : public rclcpp::Node
{
public:
  explicit LeapMotion(const rclcpp::NodeOptions & options);
  virtual ~LeapMotion();

protected:
  // ----> Initialization functions
  //void initParameters();
  void initServices();

  //void getDebugParams();
  //void getGeneralParams();
private:
  SampleListener listener;
  Controller controller;
};

#endif  // LEAPMOTION_COMPONENT_HPP_