#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class GridmapToCostmap : public rclcpp::Node
{
public:
  GridmapToCostmap() : Node("gridmap_to_costmap")
  {
    this->declare_parameter("layer", "slope_traversability");

    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      "/elevation_mapping/elevation_map_raw", rclcpp::QoS(1).best_effort(),
      std::bind(&GridmapToCostmap::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/elevation_costmap", 1);
  }

private:
  void callback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    grid_map::GridMap gridMap;
    std::string layer = this->get_parameter("layer").as_string();

    std::vector<std::string> layers = {layer};
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, gridMap, layers)) {
      RCLCPP_WARN(this->get_logger(), "Failed to convert GridMap message");
      return;
    }

    if (!gridMap.exists(layer)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Layer '%s' not found in GridMap", layer.c_str());
      return;
    }

    // toOccupancyGrid maps [dataMin, dataMax] -> [0, 100]
    // slope_traversability: 1.0 = traversable (free), 0.0 = obstacle (lethal)
    // OccupancyGrid: 0 = free, 100 = occupied
    // So invert: dataMin=1.0, dataMax=0.0
    nav_msgs::msg::OccupancyGrid occupancyGrid;
    grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, layer, 1.0, 0.0, occupancyGrid);

    pub_->publish(occupancyGrid);
  }

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GridmapToCostmap>());
  rclcpp::shutdown();
  return 0;
}
