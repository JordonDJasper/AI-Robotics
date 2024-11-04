#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class StraightLinePlanner : public rclcpp::Node
{
public:
    StraightLinePlanner()
    : Node("StraightLinePlanner")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("straight_line_path", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&StraightLinePlanner::plan_path, this));
    }

private:
    void plan_path()
    {
        // Define start and goal positions
        geometry_msgs::msg::PoseStamped start;
        start.header.frame_id = "map_frame";
        start.pose.position.x = 0.0;
        start.pose.position.y = 0.0;

        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map_frame";
        goal.pose.position.x = 5.0;
        goal.pose.position.y = 5.0;

        nav_msgs::msg::Path global_path = createPlan(start, goal);

        // Publish the computed path
        path_pub_->publish(global_path);
    }

    nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal)
    {
        nav_msgs::msg::Path global_path;
        global_path.header.stamp = this->get_clock()->now();
        global_path.header.frame_id = "map_frame";

        // Calculate the distance and increments
        double distance = std::hypot(goal.pose.position.x - start.pose.position.x,
                                     goal.pose.position.y - start.pose.position.y);
        int total_number_of_steps = static_cast<int>(distance / 0.1); // interpolation_resolution_ = 0.1
        double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_steps;
        double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_steps;

        for (int i = 0; i <= total_number_of_steps; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = start.pose.position.x + x_increment * i;
            pose.pose.position.y = start.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0; // Keeping orientation fixed
            pose.header.stamp = this->get_clock()->now();
            pose.header.frame_id = "map_frame";
            global_path.poses.push_back(pose);
        }

        return global_path;
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StraightLinePlanner>());
    rclcpp::shutdown();
    return 0;
}
