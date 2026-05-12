#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;


class ShredderController : public rclcpp::Node
{
  public:
    ShredderController()
    : Node("shredder_controller_node")
    {
    
      joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&ShredderController::joyCallback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
      
    }

  private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){
        
        auto message = geometry_msgs::msg::Twist();

        message.linear.x = msg->axes[4] * 2;   // Forward/backward
        message.linear.y = 0.0;   // Left/right (usually 0 for ground robots)
        message.linear.z = 0.0;   // Up/down (usually 0 for ground robots)

        // Angular velocity components (rad/s)
        message.angular.x = 0.0;  // Roll (usually 0 for ground robots)
        message.angular.y = 0.0;  // Pitch (usually 0 for ground robots)
        message.angular.z = msg->axes[0] * 2;

        RCLCPP_INFO(this->get_logger(), "Publishing linear x: %f, angular z: %f", message.linear.x, message.angular.z);

        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShredderController>());
  rclcpp::shutdown();
  return 0;
}