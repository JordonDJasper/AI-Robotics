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
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" //Ackermann steering for the vehicle

using namespace std::chrono_literals;

class WaypointFollower : public rclcpp::Node {
public:
   WaypointFollower()  
       : rclcpp::Node("WaypointFollower"), current_waypoint_index_(0), all_waypoints_followed_(false), path_logged_(false) {
       
       //Publisher for Ackermann drive commands *testing*
       ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);

       // Subscriber for the path from the planner
       path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
           "a_star_path", 10, std::bind(&WaypointFollower::pathCallback, this, std::placeholders::_1));
       
       // Publisher to send current status (for example, sending current waypoint index or status)
       status_pub_ = this->create_publisher<std_msgs::msg::String>("waypoint_follower_status", 10);

       // Timer to check and move towards the current waypoint
       timer_ = this->create_wall_timer(500ms, std::bind(&WaypointFollower::followWaypoint, this));
   }

private:
   //*testing this line of code*/
   rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;  // Ackermann drive publisher

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

   // Move to current waypoint utilizing the ackermann msgs *testing*
   void moveToWaypoint(const geometry_msgs::msg::PoseStamped &waypoint) {
       ackermann_msgs::msg::AckermannDriveStamped ackermann_cmd;

       //Calculate difference in position (distance)
       double dx = waypoint.pose.position.x - current_position_.x;
       double dy = waypoint.pose.position.y - current_position_.y;
       double distance = std::hypot(dx, dy);  // Euclidean distance to the waypoint

       //Speed: Proportional control based on distance to waypoint
       ackermann_cmd.drive.speed = 0.5 * distance;

       // Heading towards the waypoint (angular control)
       double angle_to_goal = std::atan2(dy, dx);
       double angle_diff = angle_to_goal - current_yaw_;  // Assuming you have current_yaw_

       // Normalize angle_diff to be between -pi and pi
       angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

       // Steering angle: Proportional control for steering
       ackermann_cmd.drive.steering_angle = std::min(0.5 * angle_diff, 0.5); // Cap steering angle if needed

       // Publish the Ackermann command
       ackermann_pub_->publish(ackermann_cmd);

       // For the sake of the testing, we will keep this commented out. 
       // // Update the current position (for simplicity, just mock it)
       // current_position_.x = waypoint.pose.position.x;
       // current_position_.y = waypoint.pose.position.y;

       // RCLCPP_INFO(this->get_logger(), "Moving towards waypoint: x = %.2f, y = %.2f", 
       //             waypoint.pose.position.x, waypoint.pose.position.y);
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
   double current_yaw_ = 0.0; // *testing*
   
};

int main(int argc, char **argv) {
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<WaypointFollower>());
   rclcpp::shutdown();
   return 0;
}