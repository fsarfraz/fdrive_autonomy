#include <iostream>
#include <cerrno>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "comms_bridge/msg/rpm_rel.hpp"
#include "comms_bridge/msg/can_status1_msg.hpp"
#include "comms_bridge/msg/can_status2_msg.hpp"
#include "comms_bridge/msg/can_status3_msg.hpp"
#include "comms_bridge/msg/can_status4_msg.hpp"
#include "comms_bridge/msg/can_status5_msg.hpp"
#include "comms_bridge/msg/status.hpp"
#include "datatypes.h"
#include "utils.cpp"
#include <cstdlib>
#include <sdbus-c++/sdbus-c++.h>
#include <fcntl.h>
#include <thread>

class VescCanNode : public rclcpp::Node {
public:
    VescCanNode() : Node("vesc_socketcan_node"), stop_listener_(false) {
        // this->declare_parameter("vesc_id", 1);
        // this->get_parameter("vesc_id", vesc_id_);
        setup_can_socket("can1");


        using std::placeholders::_1;
        rpm_sub_ = this->create_subscription<comms_bridge::msg::RpmRel>(
            "vesc/set_rpm_rel", 10, std::bind(&VescCanNode::set_rpm_cb, this, _1));


        can_status_1_pub_ = this->create_publisher<comms_bridge::msg::CanStatus1Msg>("/vesc/can_status_1", rclcpp::QoS(1));
        can_status_2_pub_ = this->create_publisher<comms_bridge::msg::CanStatus2Msg>("/vesc/can_status_2", rclcpp::QoS(1));
        can_status_3_pub_ = this->create_publisher<comms_bridge::msg::CanStatus3Msg>("/vesc/can_status_3", rclcpp::QoS(1));
        can_status_4_pub_ = this->create_publisher<comms_bridge::msg::CanStatus4Msg>("/vesc/can_status_4", rclcpp::QoS(1));
        can_status_5_pub_ = this->create_publisher<comms_bridge::msg::CanStatus5Msg>("/vesc/can_status_5", rclcpp::QoS(1));

        vesc_status_pub_ = this->create_publisher<comms_bridge::msg::Status>("/status", rclcpp::QoS(1));
            
        ping_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&VescCanNode::send_ping_command, this));

        status_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VescCanNode::status_publisher, this));

        last_time_connected_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();


        listener_thread_ = std::thread(&VescCanNode::listen_for_can_frames, this);

        status.status = comms_bridge::msg::Status::SHREDDER_DISCONNECTED;

    }

    ~VescCanNode() {
        stop_listener_ = true;
        if (listener_thread_.joinable()) listener_thread_.join();
        if (can_socket_ >= 0) close(can_socket_);
    }

    double applyDeadzone(double value, double deadzone = 0.1){
        if (std::abs(value) < deadzone)
            {return 0.0;}
        // Scale the remaining range to maintain smooth transition

        // Map from [deadzone, 1] to [0, 1]
        double scaled_value = (std::abs(value) - deadzone) / (1.0 - deadzone);
        
        double sign = (value > 0) ? 1.0 : -1.0;
        return sign * scaled_value;
    }

private:
    int can_socket_ = -1;
    // uint8_t vesc_id_ = 1;
    std::atomic<bool> stop_listener_;
    std::thread listener_thread_;
    rclcpp::TimerBase::SharedPtr ping_timer_;
    int64_t last_time_connected_ = 0;
    int ping_tries = 0;
    comms_bridge::msg::Status status;
    rclcpp::TimerBase::SharedPtr status_timer;
    bool is_disconnected = true;

    rclcpp::Subscription<comms_bridge::msg::RpmRel>::SharedPtr rpm_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    
    rclcpp::Publisher<comms_bridge::msg::CanStatus1Msg>::SharedPtr can_status_1_pub_;
    rclcpp::Publisher<comms_bridge::msg::CanStatus2Msg>::SharedPtr can_status_2_pub_;
    rclcpp::Publisher<comms_bridge::msg::CanStatus3Msg>::SharedPtr can_status_3_pub_;
    rclcpp::Publisher<comms_bridge::msg::CanStatus4Msg>::SharedPtr can_status_4_pub_;
    rclcpp::Publisher<comms_bridge::msg::CanStatus5Msg>::SharedPtr can_status_5_pub_;
    rclcpp::Publisher<comms_bridge::msg::Status>::SharedPtr vesc_status_pub_;
 

    void setup_can_socket(const std::string &iface_name) {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create CAN socket");
            return;
        }

        int flags = fcntl(can_socket_, F_GETFL, 0);
        fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

        struct ifreq ifr;
        // std::strncpy(ifr.ifr_name, iface_name.c_str(), IFNAMSIZ);
        std::strncpy(ifr.ifr_name, "can1", IFNAMSIZ-1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get interface index");
            return;
        }

        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind CAN socket");
            return;
        }

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;  // 100ms timeout
        if (setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set socket timeout");
        }

        RCLCPP_INFO(this->get_logger(), "CAN socket setup on %s", iface_name.c_str());
    }
    
    void read_can() {
        struct can_frame frame;
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes > 0) {
            uint32_t eid = frame.can_id & CAN_EFF_MASK;            
            uint8_t vesc_id = eid & 0xFF;
            uint8_t cmd_id = (eid >> 8) & 0xFF;
            
            // std::cout << "frame id:" << static_cast<int>()
            // std::cout << "vesc_id:" << static_cast<int>(vesc_id) << std::endl;
            // std::cout << "cmd_id:" << static_cast<int>(cmd_id) << std::endl;

            if (cmd_id == 19) {

                last_time_connected_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();

                ping_tries = 0;

                status.status = comms_bridge::msg::Status::SHREDDER_CONNECTED;
                //RCLCPP_INFO(this->get_logger(), "Recieved PONG COMMAND: %ld", last_time_connected_); 
 
            }
            
            if (cmd_id == 10) {
                comms_bridge::msg::CanStatus1Msg msg;
                msg.id = vesc_id;
                int32_t rpm = (frame.data[0] << 24) | (frame.data[1] << 16) | 
                              (frame.data[2] << 8) | frame.data[3];
                int16_t current = (frame.data[4] << 8) | frame.data[5];
                int16_t duty = (frame.data[6] << 8) | frame.data[7];
                
                msg.rpm = rpm;
                msg.current = current / 10.0f;
                msg.duty = duty / 1000.0f;
                
                can_status_1_pub_->publish(msg);
                
            } else if (cmd_id == 15) {
                comms_bridge::msg::CanStatus2Msg msg;
                msg.id = vesc_id;
                int32_t amp_hours = (frame.data[0] << 24) | (frame.data[1] << 16) | 
                              (frame.data[2] << 8) | frame.data[3];
                int32_t amp_hours_charged = (frame.data[4] << 24) | (frame.data[5] << 16) | 
                              (frame.data[6] << 8) | frame.data[7];

                msg.amp_hours = amp_hours / 10000.0f;
                msg.amp_hours_charged = amp_hours_charged / 10000.0f;
                
                can_status_2_pub_->publish(msg);
                
            } else if (cmd_id == 16) {
                comms_bridge::msg::CanStatus3Msg msg;
                msg.id = vesc_id;
                int32_t watt_hours = (frame.data[0] << 24) | (frame.data[1] << 16) | 
                              (frame.data[2] << 8) | frame.data[3];
                int32_t watt_hours_charged = (frame.data[4] << 24) | (frame.data[5] << 16) | 
                              (frame.data[6] << 8) | frame.data[7];

                msg.watt_hours = watt_hours / 10000.0f;
                msg.watt_hours_charged = watt_hours_charged / 10000.0f;
                
                can_status_3_pub_->publish(msg);
                
            } else if (cmd_id == 17) {
                comms_bridge::msg::CanStatus4Msg msg;
                msg.id = vesc_id;
                int16_t temp_fet = (frame.data[0] << 8) | frame.data[1];
                int16_t temp_motor = (frame.data[2] << 8) | frame.data[3];
                int16_t current_in = (frame.data[4] << 8) | frame.data[5];
                int16_t pid_pos_now = (frame.data[6] << 8) | frame.data[7];

                msg.temp_fet = temp_fet / 10.0f;
                msg.temp_motor = temp_motor / 10.0f;
                msg.current_in = current_in;
                msg.pid_pos_now = pid_pos_now / 50.0f;
                
                can_status_4_pub_->publish(msg);
                
            } else if (cmd_id == 28) {
                comms_bridge::msg::CanStatus5Msg msg;
                msg.id = vesc_id;
                int32_t tacho_value = (frame.data[0] << 24) | (frame.data[1] << 16) | 
                              (frame.data[2] << 8) | frame.data[3];
                int16_t v_in = (frame.data[4] << 8) | frame.data[5];
                msg.tacho_value = tacho_value;
                msg.v_in = v_in / 10.0f;
                
                can_status_5_pub_->publish(msg);
            }
        
        } else if (nbytes < 0) {
            
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available, this is normal for non-blocking
                return;
            } else {
                // Real error
                //RCLCPP_WARN(this->get_logger(), "CAN read error: %s", strerror(errno));
            }
        }       
    }

    void status_publisher(){
        auto status_msg = comms_bridge::msg::Status();
 
        // Set core fields
        status_msg.name = "SHREDDER";
        status_msg.status = status.status;  // Use constants
        status_msg.time = this->now().seconds();
        
        // Publish
        vesc_status_pub_->publish(status_msg);
    }

    // set_rel
    void send_rpm_command(uint8_t vesc_id,
            CAN_PACKET_ID comm_can_id, 
            float rpm_rel0 , float rpm_rel1) {

        struct can_frame frame_master = {};
        frame_master.can_id = vesc_id | (comm_can_id << 8);
        frame_master.can_id = 0x01 | (comm_can_id << 8);
        frame_master.can_id |= CAN_EFF_FLAG;
        frame_master.can_dlc = 4;
        int32_t send_index_master = 0;
        // uint8_t buffer[8] = {0};
        
        struct can_frame frame_slave = {};
        frame_slave.can_id = vesc_id | (comm_can_id << 8);
        frame_slave.can_id = 0x02 | (comm_can_id << 8);
        frame_slave.can_id |= CAN_EFF_FLAG;
        frame_slave.can_dlc = 4;
        int32_t send_index_slave = 0;

        buffer_append_float32(frame_master.data, rpm_rel0, 1e5, &send_index_master);
        buffer_append_float32(frame_slave.data, rpm_rel1, 1e5, &send_index_slave);

        if (write(can_socket_, &frame_master, sizeof(frame_master)) != sizeof(struct can_frame)) {
      //      std::cerr << "Write failed: " << std::strerror(errno) << std::endl;
            return;
        }
        
        if (write(can_socket_, &frame_slave, sizeof(frame_slave)) != sizeof(struct can_frame)) {
      //      std::cerr << "Write failed: " << std::strerror(errno) << std::endl;
            return;
        }
    }

    void restart_can_bus(){
        auto connection = sdbus::createSystemBusConnection();
        auto systemdManager = sdbus::createProxy(
            *connection, "org.freedesktop.systemd1", "/org/freedesktop/systemd1");
        std::string serviceName = "can_restart.service";

        try {
            sdbus::ObjectPath jobPath;

            systemdManager->callMethod("RestartUnit")
                .onInterface("org.freedesktop.systemd1.Manager")
                .withArguments(serviceName, "replace")
                .storeResultsTo(jobPath);

        }
        catch (const sdbus::Error &e) {
            std::cout << "Failed to Restart Service" << std::endl;
        }
         ping_tries = 0;
         //RCLCPP_INFO(this->get_logger(), "Successfully restarted CAN LINE");
    }

    void send_ping_command(){
        //RCLCPP_INFO(this->get_logger(), "Sending ping");

        struct can_frame frame = {};
        frame.can_id = 0x01 | (CAN_PACKET_PING << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 4;
        frame.data[0] = 0x01;
        
        // Non-blocking write with error handling
        ssize_t bytes_written = write(can_socket_, &frame, sizeof(frame));
        if (bytes_written != sizeof(frame)) {
            //RCLCPP_WARN(this->get_logger(), "Failed to send ping: %s", strerror(errno));
        }

        int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        int64_t elapsed = current_time - last_time_connected_;

        if (elapsed > 1000){
            ping_tries += 1;
            status.status = comms_bridge::msg::Status::SHREDDER_DISCONNECTED;
            //RCLCPP_INFO(this->get_logger(),"DIDN'T RECEIVE PONG, trying again.... (attempt %d)", ping_tries);
        }

        if (ping_tries > 3){
            
            status.status = comms_bridge::msg::Status::SHREDDER_DISCONNECTED;       

            std::thread(&VescCanNode::restart_can_bus, this).detach();
 
            //RCLCPP_WARN(this->get_logger(), "Multiple ping failures, RESTARTING CAN");
        }
    }

    // void set_duty_cb(const std_msgs::msg::Float32::SharedPtr msg) {
    //     send_vesc_command(5, static_cast<int32_t>(msg->data * 100000));  // SET_DUTY
    // }

    void set_rpm_cb(const comms_bridge::msg::RpmRel::SharedPtr msg) {
        double rpm1 = applyDeadzone(msg->rpm_rel0, 0.12);
        double rpm2 = applyDeadzone(msg->rpm_rel1, 0.12);
        send_rpm_command(0x01, CAN_SET_RPM_REL, rpm1, rpm2);
    }

    void listen_for_can_frames() {
        while (!stop_listener_) {
            read_can();
        }
    }

   

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    auto vesc_can_node = std::make_shared<VescCanNode>();

    rclcpp::spin(vesc_can_node);
    
    rclcpp::shutdown();
    return 0;
}