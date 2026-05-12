#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <cmath>
#include <algorithm>

class PathFollower : public rclcpp::Node
{
public:
  PathFollower() : Node("path_follower")
  {
    this->declare_parameter("lookahead_distance", 1.0);
    this->declare_parameter("max_linear_vel", 0.5);
    this->declare_parameter("max_angular_vel", 1.5);
    this->declare_parameter("goal_tolerance", 0.3);
    this->declare_parameter("path_timeout", 5.0);
    this->declare_parameter("control_rate", 10.0);
    this->declare_parameter("obstacle_check_distance", 0.8);  // meters ahead
    this->declare_parameter("lethal_threshold", 80);

    lookahead_ = this->get_parameter("lookahead_distance").as_double();
    max_linear_ = this->get_parameter("max_linear_vel").as_double();
    max_angular_ = this->get_parameter("max_angular_vel").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    path_timeout_ = this->get_parameter("path_timeout").as_double();
    obstacle_check_dist_ = this->get_parameter("obstacle_check_distance").as_double();
    lethal_threshold_ = this->get_parameter("lethal_threshold").as_int();
    double rate = this->get_parameter("control_rate").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

    local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/elevation_costmap", 10,
      std::bind(&PathFollower::localCostmapCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&PathFollower::controlLoop, this));
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = msg;
    path_received_time_ = this->now();
    if (msg->poses.empty()) {
      RCLCPP_INFO(this->get_logger(), "Received empty path, stopping");
    }
  }

  void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    local_costmap_ = msg;
  }

  bool getRobotPose(double& x, double& y, double& yaw)
  {
    try {
      auto transform = tf_buffer_->lookupTransform("map", "base_link",
        tf2::TimePointZero, tf2::durationFromSec(0.1));
      x = transform.transform.translation.x;
      y = transform.transform.translation.y;
      tf2::Quaternion q(
        transform.transform.rotation.x, transform.transform.rotation.y,
        transform.transform.rotation.z, transform.transform.rotation.w);
      yaw = tf2::getYaw(q);
      return true;
    } catch (const tf2::TransformException& ex) {
      return false;
    }
  }

  bool checkObstacleAhead(double rx, double ry, double yaw)
  {
    if (!local_costmap_) return false;

    // Check a point ahead of the robot in the local costmap
    double check_x = rx + obstacle_check_dist_ * std::cos(yaw);
    double check_y = ry + obstacle_check_dist_ * std::sin(yaw);

    // Transform to local costmap frame (odom) if needed
    try {
      auto transform = tf_buffer_->lookupTransform(
        local_costmap_->header.frame_id, "map",
        tf2::TimePointZero, tf2::durationFromSec(0.1));

      // Simple 2D transform of the check point
      double dx = check_x - transform.transform.translation.x;
      double dy = check_y - transform.transform.translation.y;

      // Convert to grid coordinates
      int gx = static_cast<int>((dx - local_costmap_->info.origin.position.x) / local_costmap_->info.resolution);
      int gy = static_cast<int>((dy - local_costmap_->info.origin.position.y) / local_costmap_->info.resolution);

      int w = local_costmap_->info.width;
      int h = local_costmap_->info.height;
      if (gx >= 0 && gx < w && gy >= 0 && gy < h) {
        int8_t cost = local_costmap_->data[gy * w + gx];
        return cost >= lethal_threshold_;
      }
    } catch (const tf2::TransformException&) {
      // If we can't check, don't block
    }
    return false;
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  void controlLoop()
  {
    // No path or empty path
    if (!path_ || path_->poses.empty()) {
      stopRobot();
      return;
    }

    // Check path staleness
    double age = (this->now() - path_received_time_).seconds();
    if (age > path_timeout_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Path is stale (%.1fs old), stopping", age);
      stopRobot();
      return;
    }

    double rx, ry, yaw;
    if (!getRobotPose(rx, ry, yaw)) {
      stopRobot();
      return;
    }

    // Check if at goal
    const auto& goal_pose = path_->poses.back();
    double dist_to_goal = std::hypot(
      goal_pose.pose.position.x - rx, goal_pose.pose.position.y - ry);
    if (dist_to_goal < goal_tolerance_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached!");
      stopRobot();
      return;
    }

    // Check for obstacles ahead
    if (checkObstacleAhead(rx, ry, yaw)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Obstacle detected ahead, stopping");
      stopRobot();
      return;
    }

    // Find closest point on path
    size_t closest_idx = 0;
    double closest_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path_->poses.size(); i++) {
      double d = std::hypot(path_->poses[i].pose.position.x - rx,
                            path_->poses[i].pose.position.y - ry);
      if (d < closest_dist) {
        closest_dist = d;
        closest_idx = i;
      }
    }

    // Find lookahead point (search forward from closest point)
    size_t lookahead_idx = closest_idx;
    for (size_t i = closest_idx; i < path_->poses.size(); i++) {
      double d = std::hypot(path_->poses[i].pose.position.x - rx,
                            path_->poses[i].pose.position.y - ry);
      if (d >= lookahead_) {
        lookahead_idx = i;
        break;
      }
      lookahead_idx = i;  // use furthest point if path is shorter than lookahead
    }

    double lx = path_->poses[lookahead_idx].pose.position.x;
    double ly = path_->poses[lookahead_idx].pose.position.y;

    // Pure pursuit: compute curvature
    double dx = lx - rx;
    double dy = ly - ry;
    double L = std::hypot(dx, dy);

    if (L < 0.01) {
      stopRobot();
      return;
    }

    // Angle to lookahead point relative to robot heading
    double alpha = std::atan2(dy, dx) - yaw;
    // Normalize to [-pi, pi]
    alpha = std::atan2(std::sin(alpha), std::cos(alpha));

    // Curvature: kappa = 2 * sin(alpha) / L
    double kappa = 2.0 * std::sin(alpha) / L;

    // Compute velocities
    double linear_vel = max_linear_;

    // Slow down near goal
    if (dist_to_goal < 1.0) {
      linear_vel *= dist_to_goal;
      linear_vel = std::max(linear_vel, 0.1);  // minimum creep speed
    }

    // Slow down on sharp turns
    if (std::abs(alpha) > M_PI / 4) {
      linear_vel *= 0.5;
    }

    double angular_vel = linear_vel * kappa;

    // Clamp
    linear_vel = std::clamp(linear_vel, -max_linear_, max_linear_);
    angular_vel = std::clamp(angular_vel, -max_angular_, max_angular_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_vel;
    cmd.angular.z = angular_vel;
    cmd_pub_->publish(cmd);
  }

  double lookahead_, max_linear_, max_angular_, goal_tolerance_, path_timeout_;
  double obstacle_check_dist_;
  int lethal_threshold_;

  nav_msgs::msg::Path::SharedPtr path_;
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
  rclcpp::Time path_received_time_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
