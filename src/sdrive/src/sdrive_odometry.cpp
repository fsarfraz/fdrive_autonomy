#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "comms_bridge/msg/can_status1_msg.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <optional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class SdriveOdometry : public rclcpp::Node
{
public:
    SdriveOdometry() : Node("sdrive_odometry")
    {
        // Declare parameters with defaults
        this->declare_parameter("track_separation", 0.4);        // meters
        this->declare_parameter("sprocket_circumference", 0.2);  // meters (200mm)
        this->declare_parameter("gear_ratio", 14.5);
        this->declare_parameter("erpm_to_rpm_divisor", 5.0);     // VESC pole pairs / 2
        this->declare_parameter("odom_frame", "shredder_odom");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("odom_rate_hz", 50.0);

        // Load parameters
        track_separation_ = this->get_parameter("track_separation").as_double();
        sprocket_circumference_ = this->get_parameter("sprocket_circumference").as_double();
        gear_ratio_ = this->get_parameter("gear_ratio").as_double();
        erpm_divisor_ = this->get_parameter("erpm_to_rpm_divisor").as_double();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        double odom_rate = this->get_parameter("odom_rate_hz").as_double();

        rpm_subscription_ = this->create_subscription<comms_bridge::msg::CanStatus1Msg>(
            "/vesc/can_status_1", 10,
            std::bind(&SdriveOdometry::canCallback, this, std::placeholders::_1)
        );

        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/shredder_odom", rclcpp::QoS(10));
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Timer-based odometry publishing for consistent rate
        odom_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / odom_rate),
            std::bind(&SdriveOdometry::odomTimerCallback, this)
        );

        last_odom_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), 
            "Odometry node started - track_sep: %.3fm, sprocket_circ: %.3fm, gear_ratio: %.1f",
            track_separation_, sprocket_circumference_, gear_ratio_);
    }

private:
    void canCallback(const comms_bridge::msg::CanStatus1Msg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (msg->id == 1) {
            right_track_rpm_ = msg->rpm;
            right_updated_ = true;
        } else if (msg->id == 2) {
            left_track_rpm_ = msg->rpm;
            left_updated_ = true;
        }
    }

    void odomTimerCallback()
    {
        double right_speed, left_speed;
        bool have_data;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            have_data = right_updated_ && left_updated_;
            if (!have_data) {
                // Don't publish until we have data from both tracks
                return;
            }
            right_speed = calculateTrackVelocity(right_track_rpm_);
            left_speed = calculateTrackVelocity(left_track_rpm_);
        }

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_odom_time_).seconds();
        last_odom_time_ = current_time;

        // Sanity check on dt
        if (dt <= 0.0 || dt > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Invalid dt: %.3f, skipping update", dt);
            return;
        }

        // Calculate velocities
        double linear_velocity = (right_speed + left_speed) / 2.0;
        double angular_velocity = (right_speed - left_speed) / track_separation_;

        // Update pose using differential drive kinematics
        if (std::abs(angular_velocity) < 1e-6) {
            // Moving straight
            x_ += linear_velocity * cos(theta_) * dt;
            y_ += linear_velocity * sin(theta_) * dt;
        } else {
            // Turning - use exact integration
            double delta_theta = angular_velocity * dt;
            double radius = linear_velocity / angular_velocity;
            
            x_ += radius * (sin(theta_ + delta_theta) - sin(theta_));
            y_ -= radius * (cos(theta_ + delta_theta) - cos(theta_));
            theta_ += delta_theta;
        }

        // Normalize theta to [-pi, pi]
        theta_ = atan2(sin(theta_), cos(theta_));

        publishOdometry(current_time, linear_velocity, angular_velocity);

        RCLCPP_DEBUG(this->get_logger(), 
            "v=%.2f m/s, w=%.2f rad/s, x=%.2f, y=%.2f, theta=%.2f deg",
            linear_velocity, angular_velocity, x_, y_, theta_ * 180.0 / M_PI);
    }

    void publishOdometry(const rclcpp::Time& stamp, double linear_vel, double angular_vel)
    {
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta_);

        // Publish TF if enabled
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header.stamp = stamp;
            odom_tf.header.frame_id = odom_frame_;
            odom_tf.child_frame_id = base_frame_;

            odom_tf.transform.translation.x = x_;
            odom_tf.transform.translation.y = y_;
            odom_tf.transform.translation.z = 0.0;

            odom_tf.transform.rotation.x = quat.x();
            odom_tf.transform.rotation.y = quat.y();
            odom_tf.transform.rotation.z = quat.z();
            odom_tf.transform.rotation.w = quat.w();

            tf_broadcaster_->sendTransform(odom_tf);
        }

        // Publish odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = quat.x();
        odom_msg.pose.pose.orientation.y = quat.y();
        odom_msg.pose.pose.orientation.z = quat.z();
        odom_msg.pose.pose.orientation.w = quat.w();

        // Velocity in robot frame (body frame)
        odom_msg.twist.twist.linear.x = linear_vel;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = angular_vel;

        // Covariance - tune based on your system
        // These grow with velocity to model slip
        double vel_factor = std::max(0.1, std::abs(linear_vel));
        odom_msg.pose.covariance[0] = 0.001 * vel_factor;   // x
        odom_msg.pose.covariance[7] = 0.001 * vel_factor;   // y
        odom_msg.pose.covariance[14] = 1e6;                  // z (unused)
        odom_msg.pose.covariance[21] = 1e6;                  // roll (unused)
        odom_msg.pose.covariance[28] = 1e6;                  // pitch (unused)
        odom_msg.pose.covariance[35] = 0.01 * vel_factor;   // yaw

        odom_msg.twist.covariance[0] = 0.001;   // vx
        odom_msg.twist.covariance[7] = 1e6;     // vy (unused)
        odom_msg.twist.covariance[14] = 1e6;    // vz (unused)
        odom_msg.twist.covariance[21] = 1e6;    // wx (unused)
        odom_msg.twist.covariance[28] = 1e6;    // wy (unused)
        odom_msg.twist.covariance[35] = 0.01;   // wz

        odom_publisher_->publish(odom_msg);
    }

    double calculateTrackVelocity(int erpm)
    {
        // ERPM -> mechanical RPM -> track velocity (m/s)
        double rpm = static_cast<double>(erpm) / erpm_divisor_;
        double wheel_rps = rpm / 60.0 / gear_ratio_;
        return wheel_rps * sprocket_circumference_;
    }

    // Parameters
    double track_separation_;
    double sprocket_circumference_;
    double gear_ratio_;
    double erpm_divisor_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;

    // State (protected by mutex)
    std::mutex data_mutex_;
    int right_track_rpm_ = 0;
    int left_track_rpm_ = 0;
    bool right_updated_ = false;
    bool left_updated_ = false;

    // Pose state
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    rclcpp::Time last_odom_time_;

    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<comms_bridge::msg::CanStatus1Msg>::SharedPtr rpm_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SdriveOdometry>();
    RCLCPP_INFO(node->get_logger(), "Starting odometry publisher...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}