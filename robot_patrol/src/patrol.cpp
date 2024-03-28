#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class Patrol : public rclcpp::Node
{
public:
  Patrol() : Node("robot_patrol_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&Patrol::scanCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RobotPatrol::controlLoop, this));
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int num_rays = msg->ranges.size();

    int start_index = num_rays / 4;
    int end_index = num_rays * 3 / 4;

    float largest_distance = 0.0;
    float angle = 0.0;

    for (int i = start_index; i < end_index; i++)
    {
      float distance = msg->ranges[i];

      if (std::isfinite(distance) && distance > largest_distance)
      {
        largest_distance = distance;
        angle = msg->angle_min + i * msg->angle_increment;
      }
    }

    direction_ = angle;

    RCLCPP_INFO(this->get_logger(), "Largest distance: %.2f, Angle: %.2f", largest_distance, direction_);
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist velocity_msg;
    velocity_msg.linear.x = 0.1;  
    velocity_msg.angular.z = direction_ / 2;  

    publisher_->publish(velocity_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  float direction_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}