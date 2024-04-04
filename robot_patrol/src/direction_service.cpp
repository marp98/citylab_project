#include "rclcpp/rclcpp.hpp"
#include "robot_patrol_msg/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>
#include <cmath>

using namespace std::chrono_literals;
using GetDirection = robot_patrol_msg::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node
{
public:
    DirectionService() : Node("direction_service")
    {
        RCLCPP_INFO(this->get_logger(), "DirectionService node created");
        srv_ = create_service<GetDirection>("direction_service", std::bind(&DirectionService::direction_callback, this, _1, _2));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    };

private:
    rclcpp::Service<GetDirection>::SharedPtr srv_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    geometry_msgs::msg::Twist twist_msg;

    void direction_callback(const std::shared_ptr<GetDirection::Request> request, const std::shared_ptr<GetDirection::Response> response)
    {   
        RCLCPP_INFO(this->get_logger(), "Received direction request");

        const auto& laser_data = request->laser_data;
        const int num_readings = laser_data.ranges.size();
        const double angle_increment = laser_data.angle_increment;

        // simulation
        float frontRange = laser_data.ranges[0];
        float threshold = 0.9;

        // real robot
        // float frontRange = laser_data.ranges[laser_data.ranges.size() / 2];
        // float threshold = 0.9;

        RCLCPP_INFO(this->get_logger(), "Number of laser readings: %d", num_readings);
        RCLCPP_INFO(this->get_logger(), "Angle increment: %f", angle_increment);

        // simulation
        const double left_start_angle = M_PI / 6;    // 30 degrees
        const double left_end_angle = M_PI / 2;      // 90 degrees
        const double right_start_angle = 3 * M_PI / 2;  // 270 degrees
        const double right_end_angle = 11 * M_PI / 6;   // 330 degrees

        const int left_start = static_cast<int>(left_start_angle / angle_increment);
        const int left_end = static_cast<int>(left_end_angle / angle_increment);
        const int right_start = static_cast<int>(right_start_angle / angle_increment);
        const int right_end = static_cast<int>(right_end_angle / angle_increment);

        double left_sum = 0.0;
        double front_sum = 0.0;
        double right_sum = 0.0;

        for (int i = left_start; i <= left_end; ++i) {
            double range = laser_data.ranges[i];
            left_sum += (std::isinf(range)) ? 0.0 : range;
        }
        for (int i = laser_data.ranges.size() / 2; i <= static_cast<int>(M_PI / 6 / angle_increment); ++i) {
            double range = laser_data.ranges[i];
            front_sum += (std::isinf(range)) ? 0.0 : range;
        }
        for (int i = right_end; i < num_readings; ++i) {
            double range = laser_data.ranges[i];
            front_sum += (std::isinf(range)) ? 0.0 : range;
        }
        for (int i = right_start; i <= right_end; ++i) {
            double range = laser_data.ranges[i];
            right_sum += (std::isinf(range)) ? 0.0 : range;
        }

       // Real Robot
        // int rightStartIndex = laser_data.ranges.size() / 4;
        // int rightEndIndex = laser_data.ranges.size() * 5 / 12;
        // int frontStartIndex = laser_data.ranges.size() * 5 / 12;
        // int frontEndIndex = laser_data.ranges.size() * 7 / 12;
        // int leftStartIndex = laser_data.ranges.size() * 7 / 12;
        // int leftEndIndex = laser_data.ranges.size() * 3 / 4;

        // for (int i = leftStartIndex; i <= leftEndIndex; ++i) {
        //     double range = laser_data.ranges[i];
        //     left_sum += (std::isinf(range) || range > 10.0) ? 0.0 : range;
        // }

        // for (int i = frontStartIndex; i <= frontEndIndex; ++i) {
        //     double range = laser_data.ranges[i];
        //     front_sum += (std::isinf(range) || range > 10.0) ? 0.0 : range;
        // }

        // for (int i = rightStartIndex; i <= rightEndIndex; ++i) {
        //     double range = laser_data.ranges[i];
        //     right_sum += (std::isinf(range) || range > 10.0) ? 0.0 : range;
        // }

        RCLCPP_INFO(this->get_logger(), "Left sum: %f", left_sum);
        RCLCPP_INFO(this->get_logger(), "Front sum: %f", front_sum);
        RCLCPP_INFO(this->get_logger(), "Right sum: %f", right_sum);

        if (std::isfinite(frontRange) && frontRange > threshold) {
            response->direction = "forward";  
        } else if (left_sum >= front_sum && left_sum >= right_sum) {
            response->direction = "left";
        } else if (front_sum >= left_sum && front_sum >= right_sum) {
            response->direction = "forward";
        } else {
            response->direction = "right";
        }

        RCLCPP_INFO(this->get_logger(), "Sending direction response: %s", response->direction.c_str());
    };
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "Initializing DirectionService node");
    rclcpp::spin(std::make_shared<DirectionService>());
    rclcpp::shutdown();
    return 0;
}