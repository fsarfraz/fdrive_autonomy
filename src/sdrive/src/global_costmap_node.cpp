#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <mutex>
#include <cmath>
#include <algorithm>
#include <limits>

class GlobalCostmap : public rclcpp::Node
{
public:
  GlobalCostmap() : Node("global_costmap")
  {
    // --- Parameters ---
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("map_size", 0.0);          // meters; 0 = grow from empty
    this->declare_parameter("publish_rate", 1.0);      // Hz
    this->declare_parameter("log_odds_occ", 0.85);
    this->declare_parameter("log_odds_free", -0.7);   // stronger free-space evidence vs occupied
    this->declare_parameter("log_odds_max", 5.0);
    this->declare_parameter("log_odds_min", -5.0);
    this->declare_parameter("decay_rate", 0.05);       // faster decay so stale noise fades
    this->declare_parameter("unknown_threshold", 0.35); // require more confidence before rendering
    this->declare_parameter("expansion_padding", 1.0); // extra metres added on each expansion
    this->declare_parameter("inflation_radius", 0.3);   // metres to inflate around obstacles
    this->declare_parameter("inflation_threshold", 65); // minimum cost cell to inflate from

    resolution_     = this->get_parameter("resolution").as_double();
    double map_size = this->get_parameter("map_size").as_double();
    double pub_rate = this->get_parameter("publish_rate").as_double();
    l_occ_          = this->get_parameter("log_odds_occ").as_double();
    l_free_         = this->get_parameter("log_odds_free").as_double();
    l_max_          = this->get_parameter("log_odds_max").as_double();
    l_min_          = this->get_parameter("log_odds_min").as_double();
    decay_rate_      = this->get_parameter("decay_rate").as_double();
    unknown_thresh_  = this->get_parameter("unknown_threshold").as_double();
    expansion_pad_   = this->get_parameter("expansion_padding").as_double();
    double infl_m    = this->get_parameter("inflation_radius").as_double();
    inflation_cells_ = static_cast<int>(std::round(infl_m / resolution_));
    infl_threshold_  = static_cast<int8_t>(this->get_parameter("inflation_threshold").as_int());

    grid_w_   = static_cast<int>(map_size / resolution_);
    grid_h_   = grid_w_;
    origin_x_ = -map_size / 2.0;
    origin_y_ = -map_size / 2.0;

    log_odds_grid_.assign(static_cast<size_t>(grid_w_) * grid_h_, 0.0f);

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/elevation_costmap", 10,
      std::bind(&GlobalCostmap::localCostmapCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap", 1);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / pub_rate)),
      std::bind(&GlobalCostmap::publishGlobalCostmap, this));

    RCLCPP_INFO(this->get_logger(),
      "Global costmap initialized: %dx%d cells, %.2fm res, %.1fm map, "
      "log-odds[%.2f, %.2f], decay=%.4f/cycle, expansion_pad=%.1fm, inflation=%d cells",
      grid_w_, grid_h_, resolution_, map_size, l_min_, l_max_,
      decay_rate_, expansion_pad_, inflation_cells_);
  }

private:
  void applyInflation(std::vector<int8_t>& data, int w, int h) const
  {
    if (inflation_cells_ <= 0) return;
    const std::vector<int8_t> src = data;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (src[y * w + x] < infl_threshold_) continue;
        for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
          for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
            float d = std::sqrt(static_cast<float>(dx * dx + dy * dy));
            if (d > inflation_cells_) continue;
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
            if (data[ny * w + nx] < 0) continue; // don't inflate into unknown
            float t = 1.0f - (d / static_cast<float>(inflation_cells_));
            auto inflated = static_cast<int8_t>(
              infl_threshold_ + static_cast<int>(t * (100 - infl_threshold_)));
            if (data[ny * w + nx] < inflated) data[ny * w + nx] = inflated;
          }
        }
      }
    }
  }

  float costToLogOddsUpdate(int8_t cost) const
  {
    double t = static_cast<double>(cost) / 100.0;
    return static_cast<float>(l_free_ + t * (l_occ_ - l_free_));
  }

  int8_t logOddsToCost(float lo) const
  {
    if (std::abs(lo) < unknown_thresh_) return -1;
    double prob = 1.0 / (1.0 + std::exp(-static_cast<double>(lo)));
    return static_cast<int8_t>(std::clamp(static_cast<int>(prob * 100.0), 0, 100));
  }

  // Grow the grid so it covers the given map-frame bounding box.
  // Existing data is copied into the new (larger) grid. Caller holds grid_mutex_.
  void expandGrid(double need_min_x, double need_max_x,
                  double need_min_y, double need_max_y)
  {
    double new_ox, new_oy, new_max_x, new_max_y;
    if (grid_w_ == 0 || grid_h_ == 0) {
      new_ox = need_min_x - expansion_pad_;
      new_oy = need_min_y - expansion_pad_;
      new_max_x = need_max_x + expansion_pad_;
      new_max_y = need_max_y + expansion_pad_;
    } else {
      double cur_max_x = origin_x_ + grid_w_ * resolution_;
      double cur_max_y = origin_y_ + grid_h_ * resolution_;
      new_ox = std::min(origin_x_, need_min_x - expansion_pad_);
      new_oy = std::min(origin_y_, need_min_y - expansion_pad_);
      new_max_x = std::max(cur_max_x, need_max_x + expansion_pad_);
      new_max_y = std::max(cur_max_y, need_max_y + expansion_pad_);
    }

    int new_w = static_cast<int>(std::ceil((new_max_x - new_ox) / resolution_));
    int new_h = static_cast<int>(std::ceil((new_max_y - new_oy) / resolution_));

    std::vector<float> new_grid(static_cast<size_t>(new_w) * new_h, 0.0f);

    // Where does the old origin land inside the new grid?
    int off_x = static_cast<int>(std::round((origin_x_ - new_ox) / resolution_));
    int off_y = static_cast<int>(std::round((origin_y_ - new_oy) / resolution_));

    for (int gy = 0; gy < grid_h_; ++gy) {
      for (int gx = 0; gx < grid_w_; ++gx) {
        int nx = gx + off_x;
        int ny = gy + off_y;
        if (nx >= 0 && nx < new_w && ny >= 0 && ny < new_h) {
          new_grid[ny * new_w + nx] = log_odds_grid_[gy * grid_w_ + gx];
        }
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Expanding global costmap: %dx%d -> %dx%d, origin (%.1f,%.1f) -> (%.1f,%.1f)",
      grid_w_, grid_h_, new_w, new_h, origin_x_, origin_y_, new_ox, new_oy);

    log_odds_grid_ = std::move(new_grid);
    origin_x_ = new_ox;
    origin_y_ = new_oy;
    grid_w_   = new_w;
    grid_h_   = new_h;
  }

  void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    double tx = 0.0, ty = 0.0, tyaw = 0.0;

    const std::string& source_frame = msg->header.frame_id;
    if (source_frame.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Local costmap has empty frame_id");
      return;
    }

    if (source_frame != "map") {
      try {
        auto tf = tf_buffer_->lookupTransform(
          "map", source_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
        tx = tf.transform.translation.x;
        ty = tf.transform.translation.y;
        tf2::Quaternion q(
          tf.transform.rotation.x, tf.transform.rotation.y,
          tf.transform.rotation.z, tf.transform.rotation.w);
        tyaw = tf2::getYaw(q);
      } catch (const tf2::TransformException&) {
        // No map→source transform yet (pre-loop-closure), use identity
      }
    }

    const double local_res = msg->info.resolution;
    const int    local_w   = static_cast<int>(msg->info.width);
    const int    local_h   = static_cast<int>(msg->info.height);
    const double cos_yaw   = std::cos(tyaw);
    const double sin_yaw   = std::sin(tyaw);
    const double local_ox  = msg->info.origin.position.x;
    const double local_oy  = msg->info.origin.position.y;

    // First pass: compute map-frame bounding box of all valid (non-unknown) cells
    double min_mx =  std::numeric_limits<double>::max();
    double max_mx = -std::numeric_limits<double>::max();
    double min_my =  std::numeric_limits<double>::max();
    double max_my = -std::numeric_limits<double>::max();
    bool has_valid = false;

    for (int ly = 0; ly < local_h; ++ly) {
      for (int lx = 0; lx < local_w; ++lx) {
        if (msg->data[ly * local_w + lx] < 0) continue;
        double px = local_ox + (lx + 0.5) * local_res;
        double py = local_oy + (ly + 0.5) * local_res;
        double mx = cos_yaw * px - sin_yaw * py + tx;
        double my = sin_yaw * px + cos_yaw * py + ty;
        min_mx = std::min(min_mx, mx);  max_mx = std::max(max_mx, mx);
        min_my = std::min(min_my, my);  max_my = std::max(max_my, my);
        has_valid = true;
      }
    }

    if (!has_valid) return;

    std::lock_guard<std::mutex> lock(grid_mutex_);

    // Expand if any valid data falls outside current bounds (or grid is empty)
    const double cur_max_x = origin_x_ + grid_w_ * resolution_;
    const double cur_max_y = origin_y_ + grid_h_ * resolution_;
    if (grid_w_ == 0 || grid_h_ == 0 ||
        min_mx < origin_x_ || max_mx >= cur_max_x ||
        min_my < origin_y_ || max_my >= cur_max_y) {
      expandGrid(min_mx, max_mx, min_my, max_my);
    }

    const double inv_res = 1.0 / resolution_;
    int cells_updated = 0;

    // Second pass: write into (possibly expanded) grid
    for (int ly = 0; ly < local_h; ++ly) {
      for (int lx = 0; lx < local_w; ++lx) {
        int8_t cost = msg->data[ly * local_w + lx];
        if (cost < 0) continue;

        double px = local_ox + (lx + 0.5) * local_res;
        double py = local_oy + (ly + 0.5) * local_res;
        double mx = cos_yaw * px - sin_yaw * py + tx;
        double my = sin_yaw * px + cos_yaw * py + ty;

        int gx = static_cast<int>((mx - origin_x_) * inv_res);
        int gy = static_cast<int>((my - origin_y_) * inv_res);

        if (gx < 0 || gx >= grid_w_ || gy < 0 || gy >= grid_h_) continue;

        int idx = gy * grid_w_ + gx;
        log_odds_grid_[idx] = std::clamp(
          log_odds_grid_[idx] + costToLogOddsUpdate(cost),
          static_cast<float>(l_min_),
          static_cast<float>(l_max_));
        ++cells_updated;
      }
    }

    if (cells_updated == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "No cells updated. Local costmap: %dx%d, frame='%s', "
        "origin=(%.2f,%.2f), tf=(%.2f,%.2f,%.2frad)",
        local_w, local_h, source_frame.c_str(),
        local_ox, local_oy, tx, ty, tyaw);
    }
  }

  void publishGlobalCostmap()
  {
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = this->now();
    out.header.frame_id = "map";
    out.info.resolution = static_cast<float>(resolution_);

    std::lock_guard<std::mutex> lock(grid_mutex_);

    out.info.width  = grid_w_;
    out.info.height = grid_h_;
    out.info.origin.position.x = origin_x_;
    out.info.origin.position.y = origin_y_;
    out.info.origin.orientation.w = 1.0;
    out.data.resize(static_cast<size_t>(grid_w_) * grid_h_);

    for (int i = 0, n = grid_w_ * grid_h_; i < n; ++i) {
      float& lo = log_odds_grid_[i];
      if      (lo > 0.0f) lo = std::max(0.0f, lo - static_cast<float>(decay_rate_));
      else if (lo < 0.0f) lo = std::min(0.0f, lo + static_cast<float>(decay_rate_));
      out.data[i] = logOddsToCost(lo);
    }

    applyInflation(out.data, grid_w_, grid_h_);

    pub_->publish(out);
  }

  // --- Parameters ---
  double  resolution_;
  double  origin_x_, origin_y_;
  int     grid_w_, grid_h_;
  double  l_occ_, l_free_, l_max_, l_min_;
  double  decay_rate_;
  double  unknown_thresh_;
  double  expansion_pad_;
  int     inflation_cells_;
  int8_t  infl_threshold_;

  // --- State ---
  std::vector<float> log_odds_grid_;
  std::mutex grid_mutex_;

  // --- ROS ---
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr    pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalCostmap>());
  rclcpp::shutdown();
  return 0;
}
