#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

class SimplePlanner : public rclcpp::Node
{
public:
  SimplePlanner() : Node("simple_planner")
  {
    this->declare_parameter("lethal_threshold", 80);
    this->declare_parameter("replan_on_costmap_update", true);

    lethal_threshold_ = this->get_parameter("lethal_threshold").as_int();
    replan_on_update_ = this->get_parameter("replan_on_costmap_update").as_bool();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap", 10,
      std::bind(&SimplePlanner::costmapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&SimplePlanner::goalCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
  }

private:
  struct Cell {
    int x, y;
    bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
  };

  struct CellHash {
    size_t operator()(const Cell& c) const {
      return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 16);
    }
  };

  struct AStarNode {
    Cell cell;
    double f_cost;
    bool operator>(const AStarNode& o) const { return f_cost > o.f_cost; }
  };

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    costmap_ = msg;
    if (replan_on_update_ && has_goal_) {
      planPath();
    }
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = *msg;
    has_goal_ = true;
    RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_.pose.position.x, goal_.pose.position.y);
    planPath();
  }

  bool getRobotPose(double& x, double& y)
  {
    try {
      auto transform = tf_buffer_->lookupTransform("map", "base_link",
        tf2::TimePointZero, tf2::durationFromSec(0.5));
      x = transform.transform.translation.x;
      y = transform.transform.translation.y;
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
      return false;
    }
  }

  Cell worldToGrid(double wx, double wy)
  {
    return {
      static_cast<int>((wx - costmap_->info.origin.position.x) / costmap_->info.resolution),
      static_cast<int>((wy - costmap_->info.origin.position.y) / costmap_->info.resolution)
    };
  }

  void gridToWorld(const Cell& c, double& wx, double& wy)
  {
    wx = costmap_->info.origin.position.x + (c.x + 0.5) * costmap_->info.resolution;
    wy = costmap_->info.origin.position.y + (c.y + 0.5) * costmap_->info.resolution;
  }

  bool isValid(const Cell& c)
  {
    int w = costmap_->info.width;
    int h = costmap_->info.height;
    if (c.x < 0 || c.x >= w || c.y < 0 || c.y >= h) return false;
    int8_t cost = costmap_->data[c.y * w + c.x];
    return cost >= 0 && cost < lethal_threshold_;
  }

  double heuristic(const Cell& a, const Cell& b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  void planPath()
  {
    if (!costmap_ || !has_goal_) return;

    double rx, ry;
    if (!getRobotPose(rx, ry)) return;

    Cell start = worldToGrid(rx, ry);
    Cell goal = worldToGrid(goal_.pose.position.x, goal_.pose.position.y);

    if (!isValid(start)) {
      RCLCPP_WARN(this->get_logger(), "Start cell is invalid/lethal");
      return;
    }
    if (!isValid(goal)) {
      RCLCPP_WARN(this->get_logger(), "Goal cell is invalid/lethal");
      return;
    }

    // A* search
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
    std::unordered_map<Cell, double, CellHash> g_cost;
    std::unordered_map<Cell, Cell, CellHash> came_from;

    g_cost[start] = 0.0;
    open.push({start, heuristic(start, goal)});

    // 8-connected neighbors
    const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const double dcost[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

    bool found = false;
    int iterations = 0;
    const int max_iterations = costmap_->info.width * costmap_->info.height;

    while (!open.empty() && iterations++ < max_iterations) {
      AStarNode current = open.top();
      open.pop();

      if (current.cell == goal) {
        found = true;
        break;
      }

      double current_g = g_cost[current.cell];
      if (current.f_cost > current_g + heuristic(current.cell, goal) + 1e-3) {
        continue; // stale entry
      }

      for (int i = 0; i < 8; i++) {
        Cell neighbor = {current.cell.x + dx[i], current.cell.y + dy[i]};
        if (!isValid(neighbor)) continue;

        // Add costmap cost as traversal penalty
        int8_t cell_cost = costmap_->data[neighbor.y * costmap_->info.width + neighbor.x];
        double move_cost = dcost[i] * (1.0 + cell_cost / 100.0);
        double new_g = current_g + move_cost;

        auto it = g_cost.find(neighbor);
        if (it == g_cost.end() || new_g < it->second) {
          g_cost[neighbor] = new_g;
          came_from[neighbor] = current.cell;
          open.push({neighbor, new_g + heuristic(neighbor, goal)});
        }
      }
    }

    if (!found) {
      RCLCPP_WARN(this->get_logger(), "No path found to goal (searched %d cells)", iterations);
      // Publish empty path to signal failure
      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = this->now();
      empty_path.header.frame_id = "map";
      path_pub_->publish(empty_path);
      return;
    }

    // Reconstruct path
    std::vector<Cell> path_cells;
    Cell c = goal;
    while (!(c == start)) {
      path_cells.push_back(c);
      c = came_from[c];
    }
    path_cells.push_back(start);
    std::reverse(path_cells.begin(), path_cells.end());

    // Convert to Path message
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "map";
    path.poses.reserve(path_cells.size());

    for (const auto& cell : path_cells) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      gridToWorld(cell, pose.pose.position.x, pose.pose.position.y);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Path found: %zu waypoints", path.poses.size());
    path_pub_->publish(path);
  }

  int lethal_threshold_;
  bool replan_on_update_;
  bool has_goal_ = false;
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePlanner>());
  rclcpp::shutdown();
  return 0;
}
