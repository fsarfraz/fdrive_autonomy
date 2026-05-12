
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include "comms_bridge/msg/rpm_rel.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class SdriveController: public rclcpp::Node
{
public:
    SdriveController() : Node("sdrive_controller")
    {
        cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SdriveController::cmdVelCallback, this, std::placeholders::_1)
        );

        rpm_pub_ = this->create_publisher<comms_bridge::msg::RpmRel>("/vesc/set_rpm_rel", rclcpp::QoS(10));

    }


private:
    rclcpp::Publisher<comms_bridge::msg::RpmRel>::SharedPtr rpm_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;


    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Extract desired velocities from Twist message
        double linear_vel = msg->linear.x;   // m/s (forward/backward)
        double angular_vel = msg->angular.z; // rad/s (rotation)
        
        // Differential drive inverse kinematics
        // Convert linear and angular velocity to left and right track velocities
        double track_separation = 0.4; // m (your track width)
        
        double right_track_vel = linear_vel + (angular_vel * track_separation / 2.0);
        double left_track_vel = linear_vel - (angular_vel * track_separation / 2.0);
        
        // Convert track velocities (m/s) to RPM for your motor controllers
        double right_rpm = trackVelocityToRpm(right_track_vel);
        double left_rpm = trackVelocityToRpm(left_track_vel);

        auto rpm_msg = comms_bridge::msg::RpmRel();

        right_rpm = std::clamp(right_rpm, -15000.0, 15000.0) / 15000.0;
        left_rpm = std::clamp(left_rpm, -15000.0, 15000.0) / 15000.0;
         
        rpm_msg.rpm_rel0 = right_rpm;
        rpm_msg.rpm_rel1 = left_rpm;
        RCLCPP_INFO(this->get_logger(), "Publishing right: %.3f and left: %.3f", right_rpm, left_rpm);
        
        rpm_pub_->publish(rpm_msg); 

    }
    


    double trackVelocityToRpm(double velocity_m_s) {
        // This is the inverse of your calculateTrackVelocity function
        // velocity = (RPM * 2 * pi * radius) / 60
        // So: RPM = (velocity * 60) / (2 * pi * radius)
        
        double track_radius = 0.1; // m (adjust to your actual sprocket/wheel radius)
        double rpm = (velocity_m_s * 60.0) / (2.0 * M_PI * track_radius);
        
        return rpm;
    }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SdriveController>();
    
    RCLCPP_INFO(node->get_logger(), "Starting random point cloud publisher...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}