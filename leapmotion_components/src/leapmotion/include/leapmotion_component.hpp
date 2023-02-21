#ifndef LEAPMOTION_COMPONENT_HPP_
#define LEAPMOTION_COMPONENT_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <iostream>
#include <cstring>

#include "Leap.h"
#include "sample_listener.hpp"

using namespace std::chrono_literals;
using namespace Leap;

typedef std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> markerPub;
typedef std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> markerArrayPub;

typedef std::unique_ptr<visualization_msgs::msg::Marker> markerMsgPtr;
typedef std::unique_ptr<visualization_msgs::msg::MarkerArray> markerArrayMsgPtr;

class LeapMotion : public rclcpp::Node
{
public:
  explicit LeapMotion(const rclcpp::NodeOptions & options);
  virtual ~LeapMotion();

protected:
  // ----> Initialization functions
  //void initParameters();
  void initServices();
  void initPublishers();
  bool startSensor();
  void threadFunc_leapGrab();
  void threadFunc_pubSensorsData();

  //void getDebugParams();
  //void getGeneralParams();
private:
  SampleListener listener;
  Controller controller;

  std::thread mGrabThread;        // Main grab thread
  std::thread mSensThread;        // Sensors data publish thread

  rclcpp::QoS mMappingQos;

  markerPub mPubMarker;
  markerArrayPub mPubMarkerArray;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  //void timer_callback();
};

#endif  // LEAPMOTION_COMPONENT_HPP_