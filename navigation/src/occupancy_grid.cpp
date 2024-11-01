#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class OccupancyGrid_Publisher : public rclcpp::Node
{
public:
    OccupancyGrid_Publisher()
    : Node("OccupancyGrid_Publisher")
    {
        og_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_occupancy_grid", 10);
        og_timer = this->create_wall_timer(
            500ms, std::bind(&OccupancyGrid_Publisher::og_callback, this));
    }

private:
    void og_callback()
    {
        // Create a vector with 100 elements, each initialized to 10.0f
        std::vector<float> range(100, 10.0f);

        // Initialize the occupancy grid message
        auto og_msg = nav_msgs::msg::OccupancyGrid();

        // Set the header and metadata
        og_msg.header.stamp = this->get_clock()->now();
        og_msg.header.frame_id = "map_frame";
        og_msg.info.resolution = 1.0;

        // 10 x 10 grid
        og_msg.info.width = 10;
        og_msg.info.height = 10;

        og_msg.info.origin.position.x = 0.0;
        og_msg.info.origin.position.y = 0.0;
        og_msg.info.origin.position.z = 0.0;
        og_msg.info.origin.orientation.x = 0.0;
        og_msg.info.origin.orientation.y = 0.0;
        og_msg.info.origin.orientation.z = 0.0;
        og_msg.info.origin.orientation.w = 1.0;

        // Corrected data assignment using a vector initializer
        og_msg.data.= {100, 0, 0, 0, -1, 0, 0, 0, 100}

        // Publish the occupancy grid message
        og_pub->publish(og_msg);
    }
    
    //colcon build
    //source /opt/ros/humble/setup.bash
    //ros2 run navigation occupancy_grid | Another terminal: rviz2 
    //Note: When you are putting Occupancy Grid on Rviz2, make sure tha the Fixed Frame is the same as frame_id "map_frame". 
    //Note: When you are putting Occupancy Grid on Rviz2, make sure tha the Fixed Frame is the same as frame_id "map_frame". 
    //Note: When you are putting Occupancy Grid on Rviz2, make sure tha the Fixed Frame is the same as frame_id "map_frame". 
    

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub;
    rclcpp::TimerBase::SharedPtr og_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGrid_Publisher>());
    rclcpp::shutdown();
    return 0;
}
