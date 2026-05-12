#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import time

class TrackControllerWithOdometry(Node):
    def __init__(self):
        super().__init__('track_controller')
        
        # Parameters
        self.track_width = 0.695  # Total width between tracks in meters
        self.wheel_radius = 0.0862  # Wheel radius in meters
        
        # Robot position state
        self.x = 0.0  # position in x [m]
        self.y = 0.0  # position in y [m]
        self.th = 0.0  # orientation [rad]
        
        # Velocities
        self.vx = 0.0  # linear velocity in x [m/s]
        self.vy = 0.0  # linear velocity in y [m/s] (always 0 for differential drive)
        self.vth = 0.0  # angular velocity [rad/s]
        
        # Create subscribers and publishers
        self.velocity_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10)
            
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10)
            
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            10)
            
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Track current wheel velocities
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        
        # Track wheel positions for visualization
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.last_update_time = time.time()
        
        # Create a timer to publish joint states and odometry at a fixed rate (30Hz)
        self.timer = self.create_timer(0.033, self.update_and_publish)
        
        self.get_logger().info('Track controller with odometry started')
        
    def velocity_callback(self, msg):
        # Extract linear and angular velocities
        self.vx = msg.linear.x
        self.vth = msg.angular.z
        
        # Calculate wheel velocities based on differential drive kinematics
        self.left_wheel_velocity = (self.vx - self.vth * self.track_width / 2.0) / self.wheel_radius
        self.right_wheel_velocity = (self.vx + self.vth * self.track_width / 2.0) / self.wheel_radius
        
        self.get_logger().info(f'Left: {self.left_wheel_velocity:.2f}, Right: {self.right_wheel_velocity:.2f}')
        
    def update_odometry(self, dt):
        # Update position based on velocities and elapsed time
        delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
        delta_th = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Normalize angle to be between -pi and pi
        self.th = self.normalize_angle(self.th)
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
        
    def update_and_publish(self):
        # Calculate time since last update
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Update odometry based on current velocities
        self.update_odometry(dt)
        
        # Update wheel positions
        self.left_wheel_position += self.left_wheel_velocity * dt
        self.right_wheel_position += self.right_wheel_velocity * dt
        
        # Normalize positions to stay within reasonable bounds
        self.left_wheel_position = self.left_wheel_position % (2 * math.pi)
        self.right_wheel_position = self.right_wheel_position % (2 * math.pi)
        
        # 1. Publish joint states
        self.publish_joint_states()
        
        # 2. Publish odometry message
        self.publish_odometry()
        
        # 3. Publish transform
        self.publish_tf()
        
    def publish_joint_states(self):
        # Create and publish joint states message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Add all wheel joints
        joint_state_msg.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
            'left_road_wheel_front_joint',
            'left_road_wheel_rear_joint',
            'right_road_wheel_front_joint',
            'right_road_wheel_rear_joint'
        ]
        
        # Add positions for visualization
        joint_state_msg.position = [
            self.left_wheel_position,    # front_left_wheel
            self.right_wheel_position,   # front_right_wheel
            self.left_wheel_position,    # rear_left_wheel
            self.right_wheel_position,   # rear_right_wheel
            self.left_wheel_position,    # left_road_wheel_front
            self.left_wheel_position,    # left_road_wheel_rear
            self.right_wheel_position,   # right_road_wheel_front
            self.right_wheel_position    # right_road_wheel_rear
        ]
        
        # Set velocities
        joint_state_msg.velocity = [
            self.left_wheel_velocity,   # front_left_wheel
            self.right_wheel_velocity,  # front_right_wheel
            self.left_wheel_velocity,   # rear_left_wheel
            self.right_wheel_velocity,  # rear_right_wheel
            self.left_wheel_velocity,   # left_road_wheel_front
            self.left_wheel_velocity,   # left_road_wheel_rear
            self.right_wheel_velocity,  # right_road_wheel_front
            self.right_wheel_velocity   # right_road_wheel_rear
        ]
        
        # Publish the joint states
        self.joint_state_publisher.publish(joint_state_msg)
        
    def publish_odometry(self):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (convert euler angle to quaternion)
        odom.pose.pose.orientation = self.euler_to_quaternion(0, 0, self.th)
        
        # Set velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # Publish odometry message
        self.odom_publisher.publish(odom)
        
    def publish_tf(self):
        # Create transform message
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Set rotation
        q = self.euler_to_quaternion(0, 0, self.th)
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        quaternion = Quaternion()
        quaternion.x = qx
        quaternion.y = qy
        quaternion.z = qz
        quaternion.w = qw
        
        return quaternion

def main(args=None):
    rclpy.init(args=args)
    controller = TrackControllerWithOdometry()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()