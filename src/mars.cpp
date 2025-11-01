#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <rclcpp/logging.hpp>
#include <vector>

#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/detail/bool__struct.hpp>

using namespace std::chrono_literals;

struct GoalPose {
    std::string name;
    float x;
    float y;
    float radiusMin;
    float radiusMax;
};

std::pair<float, float> getRandomPoint(GoalPose &g) {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float> angleDist(0.0f, 2 * M_PI);
    std::uniform_real_distribution<float> radiusDist(g.radiusMin, g.radiusMax);

    float randAngle = angleDist(gen);
    float randRadius = radiusDist(gen);

    float newX = g.x + randRadius * std::cos(randAngle);
    float newY = g.y + randRadius * std::sin(randAngle);

    return {newX, newY};
}

geometry_msgs::msg::PoseStamped createGoal(GoalPose &g) {
    geometry_msgs::msg::PoseStamped goal_pose;
    std::pair<float, float> randPose;

    goal_pose.header.frame_id = g.name;
    randPose = getRandomPoint(g);

    goal_pose.pose.position.x = randPose.first;
    goal_pose.pose.position.y = randPose.second;
    goal_pose.pose.orientation.w = 1.0;

    return goal_pose;
}

class MarsNode : public rclcpp::Node {
  public:
    MarsNode() : Node("mars_node") {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mars_goal_poses", rclcpp::QoS(10));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rover/odom", rclcpp::QoS(10),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                this->odom_cb_(msg);
            });
        success_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/success_topic", rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->success_cb_(msg);
            });

        timer_ =
            this->create_wall_timer(100ms, [this]() { this->control_loop(); });

        original_poses_.push_back({"aruco_0", 2.9, 2.3, 0, 0});
        original_poses_.push_back({"aruco_1", 4.00, 156.1, 5, 10});
        original_poses_.push_back({"hammer", 135.4, 165.3, 1, 3});
        original_poses_.push_back({"aruco_2", 163, 69.3, 10, 20});
        original_poses_.push_back({"bottle", 145.5, 125.3, 5, 10});
        original_poses_.push_back({"gnss_1", 54.7, 149.1, 0, 0});
        original_poses_.push_back({"gnss_2", 30.5, 67.15, 0, 0});

        for (auto &gPose : original_poses_)
            mission_poses.push_back(createGoal(gPose));
    }

  private:
    void odom_cb_(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg_ = msg;
    }

    void success_cb_(const std_msgs::msg::Bool::SharedPtr msg) {
        bool_msg_ = msg;
    }

    void control_loop() {
        pose_pub_->publish(mission_poses[state_]);

        GoalPose original = original_poses_[state_];

        float c_x = odom_msg_->pose.pose.position.x;
        float c_y = odom_msg_->pose.pose.position.y;

        float error = std::sqrt((c_x - original.x) * (c_x - original.x) +
                                (c_y - original.y) * (c_y - original.y));

        if (error > 3.0) {
            RCLCPP_INFO(this->get_logger(), "Searching %s... Error : %f",
                        original.name.c_str(), error);
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "Search for %s Complete! Continiung to %s step",
                        original.name.c_str(),
                        original_poses_[state_ + 1].name.c_str());
            state_++;
        }
        // a
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr success_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> mission_poses;
    std::vector<GoalPose> original_poses_;

    nav_msgs::msg::Odometry::SharedPtr odom_msg_;
    std_msgs::msg::Bool::SharedPtr bool_msg_;

    int state_{0};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto mars_node = std::make_shared<MarsNode>();
    rclcpp::spin(mars_node);
    rclcpp::shutdown();
    return 0;
}
