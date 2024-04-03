#include "rclcpp/rclcpp.hpp"
#include "robot_patrol_msg/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using GetDirection = robot_patrol_msg::srv::GetDirection;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_service");
  rclcpp::Client<GetDirection>::SharedPtr client =
    node->create_client<GetDirection>("direction_service");

  auto request = std::make_shared<GetDirection::Request>();
  auto laser_data = std::make_shared<sensor_msgs::msg::LaserScan>();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub =
    node->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, [&](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        *laser_data = *msg;
      });

  while (laser_data->ranges.empty()) {
    rclcpp::spin_some(node);
  }

  request->laser_data = *laser_data;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Direction: %s", result->direction.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /direction_service");
  }

  rclcpp::shutdown();
  return 0;
}