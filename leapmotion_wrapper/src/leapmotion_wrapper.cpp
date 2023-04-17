#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>
#include "leapmotion_components/leapmotion_component.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto leapmotiob_node = std::make_shared<LeapMotion>(options);
  rclcpp::spin(leapmotiob_node);
  rclcpp::shutdown();
}