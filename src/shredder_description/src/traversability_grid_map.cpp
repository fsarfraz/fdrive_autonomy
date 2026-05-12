#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <cmath>

class TraversabilityComputer : public rclcpp::Node
{
public:
  TraversabilityComputer() : Node("traversability_computer")
  {
    // Declare parameters
    this->declare_parameter("slope_lower_threshold_deg", 15.0);
    this->declare_parameter("slope_critical_deg", 35.0);
    this->declare_parameter("input_topic", "/elevation_mapping_node/elevation_map_raw");
    this->declare_parameter("output_topic", "/elevation_mapping/traversability_map");
    
    // Get parameters
    slope_lower_rad_ = degToRad(this->get_parameter("slope_lower_threshold_deg").as_double());
    slope_critical_rad_ = degToRad(this->get_parameter("slope_critical_deg").as_double());
    
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    
    // Create subscriber and publisher
    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
      input_topic, 10,
      std::bind(&TraversabilityComputer::elevationCallback, this, std::placeholders::_1)
    );
    
    pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(output_topic, 10);
    
    RCLCPP_INFO(this->get_logger(), "Traversability computer started");
    RCLCPP_INFO(this->get_logger(), "Slope thresholds: %.1f° - %.1f°", 
                radToDeg(slope_lower_rad_), radToDeg(slope_critical_rad_));
  }

private:
  void elevationCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
  {
    // Convert ROS message to grid_map
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);
    
    // Check if elevation layer exists
    if (!map.exists("elevation")) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No elevation layer found!");
      return;
    }
    
    // Compute slope
    computeSlope(map);
    
    // Compute traversability from slope
    computeTraversability(map);
    
    // Convert back to ROS message and publish
    auto output_msg = grid_map::GridMapRosConverter::toMessage(map);
    pub_->publish(*output_msg);
  }
  
  void computeSlope(grid_map::GridMap& map)
{
  // Add slope layer
  map.add("slope");
  
  const auto& elevation = map["elevation"];
  auto& slope = map["slope"];
  
  const double resolution = map.getResolution();
  const grid_map::Size grid_size = map.getSize();
  
  // Iterate through all cells
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    
    // Skip if elevation is invalid
    if (!map.isValid(index, "elevation")) {
      slope(index(0), index(1)) = NAN;
      continue;
    }
    
    const double center_height = elevation(index(0), index(1));
    
    // Compute gradient using central differences
    double dx = 0.0, dy = 0.0;
    bool valid_dx = false, valid_dy = false;
    
    // X gradient (check left and right neighbors)
    grid_map::Index left_index(index(0) - 1, index(1));
    grid_map::Index right_index(index(0) + 1, index(1));
    
    // Check bounds AND validity
    bool left_valid = (left_index(0) >= 0) && 
                      map.isValid(left_index, "elevation");
    bool right_valid = (right_index(0) < grid_size(0)) && 
                       map.isValid(right_index, "elevation");
    
    if (left_valid && right_valid) {
      // Central difference
      dx = (elevation(right_index(0), right_index(1)) - 
            elevation(left_index(0), left_index(1))) / (2.0 * resolution);
      valid_dx = true;
    } else if (right_valid) {
      // Forward difference
      dx = (elevation(right_index(0), right_index(1)) - center_height) / resolution;
      valid_dx = true;
    } else if (left_valid) {
      // Backward difference
      dx = (center_height - elevation(left_index(0), left_index(1))) / resolution;
      valid_dx = true;
    }
    
    // Y gradient (check top and bottom neighbors)
    grid_map::Index top_index(index(0), index(1) - 1);
    grid_map::Index bottom_index(index(0), index(1) + 1);
    
    // Check bounds AND validity
    bool top_valid = (top_index(1) >= 0) && 
                     map.isValid(top_index, "elevation");
    bool bottom_valid = (bottom_index(1) < grid_size(1)) && 
                        map.isValid(bottom_index, "elevation");
    
    if (top_valid && bottom_valid) {
      // Central difference
      dy = (elevation(bottom_index(0), bottom_index(1)) - 
            elevation(top_index(0), top_index(1))) / (2.0 * resolution);
      valid_dy = true;
    } else if (bottom_valid) {
      // Forward difference
      dy = (elevation(bottom_index(0), bottom_index(1)) - center_height) / resolution;
      valid_dy = true;
    } else if (top_valid) {
      // Backward difference
      dy = (center_height - elevation(top_index(0), top_index(1))) / resolution;
      valid_dy = true;
    }
    
    // Compute slope magnitude
    if (valid_dx || valid_dy) {
      const double gradient_magnitude = std::sqrt(dx * dx + dy * dy);
      slope(index(0), index(1)) = std::atan(gradient_magnitude);
    } else {
      slope(index(0), index(1)) = NAN;
    }
  }
}
  
  void computeTraversability(grid_map::GridMap& map)
  {
    // Add traversability layer
    map.add("traversability");
    
    const auto& slope = map["slope"];
    auto& traversability = map["traversability"];
    
    // Iterate through all cells
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
      const grid_map::Index index(*iterator);
      
      // Skip if slope is invalid
      if (!map.isValid(index, "slope")) {
        traversability(index(0), index(1)) = NAN;
        continue;
      }
      
      const double slope_value = slope(index(0), index(1));
      
      // Compute traversability score
      if (slope_value < slope_lower_rad_) {
        // Fully traversable
        traversability(index(0), index(1)) = 1.0;
      } else if (slope_value > slope_critical_rad_) {
        // Not traversable
        traversability(index(0), index(1)) = 0.0;
      } else {
        // Linear interpolation
        const double range = slope_critical_rad_ - slope_lower_rad_;
        traversability(index(0), index(1)) = 1.0 - (slope_value - slope_lower_rad_) / range;
      }
    }
  }
  
  inline double degToRad(double deg) const { return deg * M_PI / 180.0; }
  inline double radToDeg(double rad) const { return rad * 180.0 / M_PI; }
  
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  
  double slope_lower_rad_;
  double slope_critical_rad_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TraversabilityComputer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}