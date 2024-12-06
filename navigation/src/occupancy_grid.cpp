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
        og_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_occupancy_grid", 50);
        og_timer = this->create_wall_timer(10s, std::bind(&OccupancyGrid_Publisher::og_callback, this));
    }

private:
    void og_callback()
    {
        // Initialize occupancy grid message
        auto og_msg = nav_msgs::msg::OccupancyGrid();

        // Set the header and metadata
        og_msg.header.stamp = this->get_clock()->now();
        og_msg.header.frame_id = "map_frame";
        og_msg.info.resolution = 1.0;

        // Set grid size (10x10 grid)
        og_msg.info.width = 100;
        og_msg.info.height = 100;

        // Set grid origin
        og_msg.info.origin.position.x = 0.0;
        og_msg.info.origin.position.y = 0.0;
        og_msg.info.origin.position.z = 0.0;
        og_msg.info.origin.orientation.x = 0.0;
        og_msg.info.origin.orientation.y = 0.0;
        og_msg.info.origin.orientation.z = 0.0;
        og_msg.info.origin.orientation.w = 1.0;


    //colcon build
    //source /opt/ros/humble/setup.bash
    //ros2 run navigation occupancy_grid | Another terminal: rviz2 
    //Note: When you are putting Occupancy Grid on Rviz2, make sure tha the Fixed Frame is the same as frame_id "map_frame". 
    //Note: When you are putting Occupancy Grid on Rviz2, make sure tha the Fixed Frame is the same as frame_id "map_frame". 
    //Note: When you are putting Occupancy Grid on Rviz2, make sure tha the Fixed Frame is the same as frame_id "map_frame". 


        // Ensure the data array has the correct size (width * height)
        og_msg.data.resize(og_msg.info.width * og_msg.info.height, 0);  // Initialize with 0 (free space)

        // Example: Modify some grid values
        og_msg.data[0] = 100;
        og_msg.data[101] = 100;
        og_msg.data[102] = 100;
        og_msg.data[103] = 100;   // Set the first cell to occupied (100)
        og_msg.data[104] = 100;
        og_msg.data[201] = 100;
        og_msg.data[302] = 100;
        og_msg.data[300] = 100;
        og_msg.data[202] = 100;
        og_msg.data[99] = 0;  // Set the last cell to occupied (100)

        // Optionally, you can fill the grid with random values between 0 and 100
        // for testing purposes, like:
        // for (size_t i = 0; i < og_msg.data.size(); ++i)
        // {
        //     og_msg.data[i] = rand() % 2 == 0 ? 0 : 100;
        // }

        // Publish the occupancy grid message
        og_pub->publish(og_msg);
    }

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
