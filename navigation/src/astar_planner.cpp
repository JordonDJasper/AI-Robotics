#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <unordered_set>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

struct Pose {
    double x, y;
    Pose(double x = 0.0, double y = 0.0) : x(x), y(y) {}
    bool operator==(const Pose &other) const {
        return std::fabs(x - other.x) < 1e-5 && std::fabs(y - other.y) < 1e-5;
    }
};

// Custom hash function for unordered_set
struct PoseHash {
    std::size_t operator()(const Pose &pose) const {
        return std::hash<double>()(pose.x) ^ std::hash<double>()(pose.y);
    }
};

class StraightLinePlanner : public rclcpp::Node {
public:
    StraightLinePlanner()  
        : rclcpp::Node("StraightLinePlanner"), occupancy_grid_ready_(false) {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("a_star_path", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&StraightLinePlanner::plan_path, this));
        occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "custom_occupancy_grid", 10,
            std::bind(&StraightLinePlanner::occupancyGridCallback, this, std::placeholders::_1));
    }

private:
    struct Node {
        Pose pose;
        double cost;  // Cost from start to this node
        double priority;  // f(x) = cost + heuristic
        std::shared_ptr<Node> parent;

        Node(Pose p, double c, double prio, std::shared_ptr<Node> par = nullptr)
            : pose(p), cost(c), priority(prio), parent(par) {}
    };

    struct ComparePriority {
        bool operator()(const std::shared_ptr<Node> &a, const std::shared_ptr<Node> &b) const {
            return a->priority > b->priority;
        }
    };

    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        occupancy_grid_ = *msg;
        occupancy_grid_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Occupancy grid received.");
    }

    void plan_path() {
        if (!occupancy_grid_ready_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for occupancy grid...");
            return;
        }

        geometry_msgs::msg::PoseStamped start, goal;
        start.header.frame_id = "map_frame";
        start.pose.position.x = 0.0;
        start.pose.position.y = 0.0;

        goal.header.frame_id = "map_frame";
        goal.pose.position.x = 5.0;
        goal.pose.position.y = 5.0;

        nav_msgs::msg::Path global_path = aStarSearch(start, goal);
        path_pub_->publish(global_path);
    }

    nav_msgs::msg::Path aStarSearch(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        nav_msgs::msg::Path global_path;
        global_path.header.stamp = this->get_clock()->now();
        global_path.header.frame_id = "map_frame";

        Pose start_pose(start.pose.position.x, start.pose.position.y);
        Pose goal_pose(goal.pose.position.x, goal.pose.position.y);

        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, ComparePriority> fringe;
        std::unordered_set<Pose, PoseHash> closed;

        fringe.push(std::make_shared<Node>(start_pose, 0.0, heuristic(start_pose, goal_pose)));

        while (!fringe.empty()) {
            auto current_node = fringe.top();
            fringe.pop();

            if (closed.find(current_node->pose) != closed.end()) continue;
            closed.insert(current_node->pose);

            if (current_node->pose == goal_pose) {
                reconstructPath(global_path, current_node);
                return global_path;
            }

            for (const auto &neighbor : getNeighbors(current_node->pose, closed)) {
                double cost = current_node->cost + distance(current_node->pose, neighbor);
                auto new_node = std::make_shared<Node>(neighbor, cost, cost + heuristic(neighbor, goal_pose), current_node);
                fringe.push(new_node);
            }
        }

        RCLCPP_WARN(this->get_logger(), "No path found to the goal.");
        return global_path;
    }

    std::vector<Pose> getNeighbors(const Pose &pose, const std::unordered_set<Pose, PoseHash> &closed) {
        std::vector<Pose> neighbors;
        std::vector<Pose> possible_moves = {
            {pose.x + 1.0, pose.y}, {pose.x - 1.0, pose.y},
            {pose.x, pose.y + 1.0}, {pose.x, pose.y - 1.0},
            {pose.x + 1.0, pose.y + 1.0}, {pose.x - 1.0, pose.y - 1.0},  // Diagonal moves
            {pose.x + 1.0, pose.y - 1.0}, {pose.x - 1.0, pose.y + 1.0}
        };

        for (const auto &move : possible_moves) {
            if (isValid(move) && closed.find(move) == closed.end()) {
                neighbors.push_back(move);
            }
        }
        return neighbors;
    }

    bool isValid(const Pose &pose) {
        if (!occupancy_grid_ready_) return false;

        int width = occupancy_grid_.info.width;
        int height = occupancy_grid_.info.height;
        double resolution = occupancy_grid_.info.resolution;

        int grid_x = static_cast<int>(pose.x / resolution);
        int grid_y = static_cast<int>(pose.y / resolution);

        if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) return false;

        int index = grid_y * width + grid_x;
        return occupancy_grid_.data[index] == 0;  // Free cell (0 means free)
    }

    double distance(const Pose &a, const Pose &b) const {
        return std::hypot(b.x - a.x, b.y - a.y);
    }

    double heuristic(const Pose &a, const Pose &b) const {
        return distance(a, b);  // Using Euclidean distance as heuristic
    }

    void reconstructPath(nav_msgs::msg::Path &path, std::shared_ptr<Node> node) {
        while (node) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = node->pose.x;
            pose.pose.position.y = node->pose.y;
            pose.pose.orientation.w = 1.0;
            pose.header.stamp = this->get_clock()->now();
            pose.header.frame_id = "map_frame";
            path.poses.insert(path.poses.begin(), pose);  // Insert at the beginning to reverse path order
            node = node->parent;
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    bool occupancy_grid_ready_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StraightLinePlanner>());
    rclcpp::shutdown();
    return 0;
}
