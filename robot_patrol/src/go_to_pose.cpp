#include "robot_patrol/action/go_to_pose.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/qos.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <functional>
#include <memory>
#include <thread>
#include <cmath>

class GoToPoseNode : public rclcpp::Node {
public:
    using GoToPoseAction = robot_patrol::action::GoToPose;
    using GoalHandleMovement = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

    explicit GoToPoseNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("go_to_pose_node", options) {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<GoToPoseAction>(
            this,
            "go_to_pose",
            std::bind(&GoToPoseNode::handle_goal, this, _1, _2),
            std::bind(&GoToPoseNode::handle_cancel, this, _1),
            std::bind(&GoToPoseNode::handle_accepted, this, _1)
        );

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        qos.durability_volatile();

        odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos, std::bind(&GoToPoseNode::odomCallback, this, _1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

private:
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    geometry_msgs::msg::Pose2D desired_pos_;
    geometry_msgs::msg::Pose2D current_pos_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GoToPoseAction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with position: (x: %f, y: %f, theta: %f)", goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);

        desired_pos_ = goal->goal_pos;

        (void)uuid;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMovement> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

        (void)goal_handle;

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle) {
        using namespace std::placeholders;

        std::thread{std::bind(&GoToPoseNode::execute, this, _1), goal_handle}.detach();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        tf2::Quaternion quat(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        current_pos_.x = x;
        current_pos_.y = y;
        current_pos_.theta = yaw;
    }

    void execute(const std::shared_ptr<GoalHandleMovement> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<GoToPoseAction::Feedback>();
        auto &current_pos = feedback->current_pos;
        auto result = std::make_shared<GoToPoseAction::Result>();
        geometry_msgs::msg::Twist cmd_vel;
        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->status = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            current_pos = current_pos_;

            double dx = desired_pos_.x - current_pos.x;
            double dy = desired_pos_.y - current_pos.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < 0.1) {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;

                cmd_vel_publisher_->publish(cmd_vel);

                result->status = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                return;
            }

            double direction_x = dx / distance;
            double direction_y = dy / distance;

            double angular_speed = std::atan2(direction_y, direction_x) - current_pos.theta;
            angular_speed = std::fmod(angular_speed + M_PI, 2 * M_PI) - M_PI;

            cmd_vel.linear.x = 0.2;
            cmd_vel.angular.z = angular_speed;

            cmd_vel_publisher_->publish(cmd_vel);

            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<GoToPoseNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
