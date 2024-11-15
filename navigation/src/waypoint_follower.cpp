#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class WaypointFollower : public rclcpp::Node {
public:
    WaypointFollower()  
        : rclcpp::Node("WaypointFollower"), current_waypoint_index_(0), all_waypoints_followed_(false), path_logged_(false) {
        // Subscriber for the path from the planner
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "a_star_path", 10, std::bind(&WaypointFollower::pathCallback, this, std::placeholders::_1));
        
        // Publisher to send current status (for example, sending current waypoint index or status)
        status_pub_ = this->create_publisher<std_msgs::msg::String>("waypoint_follower_status", 10);

        // Timer to check and move towards the current waypoint
        timer_ = this->create_wall_timer(500ms, std::bind(&WaypointFollower::followWaypoint, this));
    }

private:
    // Callback when the new path is received from the planner
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = msg->poses;
        if (!path_logged_) {
            RCLCPP_INFO(this->get_logger(), "Path received with %zu waypoints.", path_.size());
            path_logged_ = true; // Ensure the message is only logged once
        }
    }

    // Follow the current waypoint and move towards it
    void followWaypoint() {
        if (path_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path available to follow.");
            return;
        }

        if (current_waypoint_index_ < path_.size()) {
            geometry_msgs::msg::PoseStamped current_waypoint = path_[current_waypoint_index_];

            // Logic to move towards the waypoint (you can implement your own control system here)
            moveToWaypoint(current_waypoint);

            // Send the current status
            std_msgs::msg::String status_msg;
            status_msg.data = "Moving to waypoint " + std::to_string(current_waypoint_index_);
            status_pub_->publish(status_msg);

            // Move to the next waypoint if we are close enough to the current one
            if (isCloseToWaypoint(current_waypoint)) {
                current_waypoint_index_++;
            }
        } else if (!all_waypoints_followed_) {
            // Only print this once after all waypoints are followed
            RCLCPP_INFO(this->get_logger(), "All waypoints have been followed.");
            all_waypoints_followed_ = true;  // Set the flag to prevent future printing
        }
    }

    // Function to check if we are close enough to the current waypoint
    bool isCloseToWaypoint(const geometry_msgs::msg::PoseStamped &waypoint) {
        double tolerance = 0.1;  // Tolerance distance (meters)
        double dx = current_position_.x - waypoint.pose.position.x;
        double dy = current_position_.y - waypoint.pose.position.y;
        return std::hypot(dx, dy) < tolerance;
    }

    // Dummy function to simulate robot movement towards a waypoint
    void moveToWaypoint(const geometry_msgs::msg::PoseStamped &waypoint) {
        // Simulate movement logic (e.g., controlling velocity, turning, etc.)
        // This could involve sending messages to a velocity topic or similar

        // Update the current position (for simplicity, just mock it)
        current_position_.x = waypoint.pose.position.x;
        current_position_.y = waypoint.pose.position.y;

        RCLCPP_INFO(this->get_logger(), "Moving towards waypoint: x = %.2f, y = %.2f", 
                    waypoint.pose.position.x, waypoint.pose.position.y);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> path_;  // Path received from the planner
    size_t current_waypoint_index_;  // Index of the current waypoint to follow
    bool all_waypoints_followed_;  // Flag to track if the "All waypoints followed" message has been printed
    bool path_logged_;  // Flag to ensure the path received message is logged only once

    struct Position {
        double x = 0.0, y = 0.0;
    } current_position_;  // Mock current position of the robot
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointFollower>());
    rclcpp::shutdown();
    return 0;
}
