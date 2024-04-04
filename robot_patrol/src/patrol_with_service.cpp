#include <rclcpp/rclcpp.hpp>
#include "robot_patrol_msg/srv/get_direction.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

using GetDirection = robot_patrol_msg::srv::GetDirection;

class Patrol : public rclcpp::Node {
public:
    Patrol() : Node("robot_patrol_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::scanCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        client_ = this->create_client<GetDirection>("direction_service");
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_laser_ = msg;
    }

    void publishVelocity(double linear_x, double linear_y) {
        geometry_msgs::msg::Twist velocity_msg;
        velocity_msg.linear.x = linear_x;
        velocity_msg.angular.z = linear_y;
        publisher_->publish(velocity_msg);
    }

    void callDirectionService() {
        if (!last_laser_) {
            RCLCPP_ERROR(get_logger(), "Laser data not yet received.");
            return;
        }

        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *last_laser_;

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
        }

        auto result_future = client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto result = result_future.get();
            if (result->direction == "forward") {
                publishVelocity(0.1, 0.0);
            } else if (result->direction == "left") {
                publishVelocity(0.1, 0.5);
            } else if (result->direction == "right") {
                publishVelocity(0.1, -0.5);
            } else {
                RCLCPP_INFO(get_logger(), "Something went wrong");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to call service /direction_service");
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    sensor_msgs::msg::LaserScan::SharedPtr last_laser_;
    rclcpp::Client<GetDirection>::SharedPtr client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::Rate loop_rate(10); 

    while (rclcpp::ok()) {
        node->callDirectionService();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}