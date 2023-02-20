#include <sstream>
#include <type_traits>
#include <vector>

#include "leapmotion_component.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace Leap;

LeapMotion::LeapMotion(const rclcpp::NodeOptions & options)
: Node("leapmotion_node", options)
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "      LeapMotion Component ");
  RCLCPP_INFO(get_logger(), "********************************");

  controller.addListener(listener);
  // Parameters initialization
  //initParameters();

  // Init services
  initServices();
  initPublishers();
  startSensor();
}


LeapMotion::~LeapMotion()
{
  RCLCPP_DEBUG(get_logger(), "Destroying node");
}


void LeapMotion::initServices()
{
  RCLCPP_INFO(get_logger(), "*** SERVICES ***");
}


void LeapMotion::initPublishers()
{
  RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

}


bool LeapMotion::startSensor()
{
  RCLCPP_INFO(get_logger(), "***** STARTING CAMERA *****");

  //controller.addListener(listener);
  // Start grab thread
  mGrabThread = std::thread(&LeapMotion::threadFunc_leapGrab, this);

  if (mGrabThread.joinable()) {
    mGrabThread.join();
  }
}


void LeapMotion::threadFunc_leapGrab()
{
  RCLCPP_DEBUG(get_logger(), "Grab thread started");

  while (1) {
    //RCLCPP_INFO(get_logger(), "threadFunc_leapGrab() loop");
    ;
  }
}
