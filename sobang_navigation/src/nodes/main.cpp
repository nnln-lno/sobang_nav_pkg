#include "sobang_navigation/navigation.hpp"
#include "sobang_navigation/uwbLocalizer.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto nav_node = std::make_shared<navigation::Navigation>();
  auto uwb_node = std::make_shared<navigation::UWBLocalizer>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(nav_node);
  executor.add_node(uwb_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
} 