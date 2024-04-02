#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("robot_patrol_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::scanCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Patrol::controlLoop, this));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float frontRange = msg->ranges[0];
        float threshold = 1.0;  

        if (std::isfinite(frontRange) && frontRange > threshold) {
            direction_ = 0.0;  
        } else {
            float maxRange = 0.0;
            int maxRangeIndex = -1;
            int leftIndex = (int)(M_PI_2 / msg->angle_increment);
            int rightIndex = (int)((3 * M_PI_2) / msg->angle_increment);

            for (int i = 0; i <= leftIndex; ++i) {
                float range = msg->ranges[i];
                if (std::isfinite(range) && range > maxRange) {
                    maxRange = range;
                    maxRangeIndex = i;
                }
            }

            for (int i = rightIndex; i < msg->ranges.size(); ++i) {
                float range = msg->ranges[i];
                if (std::isfinite(range) && range > maxRange) {
                    maxRange = range;
                    maxRangeIndex = i;
                }
            }

            if (maxRangeIndex >= 0) {
                float angle = msg->angle_min + maxRangeIndex * msg->angle_increment;
                if (angle > M_PI) {
                    angle -= 2 * M_PI;
                }
                direction_ = angle;
            }
        }
    }

    void controlLoop() {
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

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();
    return 0;
}