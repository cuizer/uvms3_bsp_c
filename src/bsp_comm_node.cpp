#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "hal/msg/hal_inertialnavi.hpp"
#include "hal/msg/hal_dvl.hpp"
#include "hal/msg/hal_depthsensor.hpp"
#include "hal/msg/hal_mainthruster.hpp"
#include "hal/msg/hal_auxithruster.hpp"
#include "hal/msg/hal_battery.hpp"
#include "hal/msg/hal_tailservo.hpp"
#include "hal/msg/hal_armmotor.hpp"
#include "hal/msg/hal_antenna.hpp"


#include "hal/msg/hal_antenna_control.hpp"
#include "hal/msg/hal_light_control.hpp"
#include "hal/msg/hal_mode_control.hpp"
#include "hal/msg/hal_remote_control.hpp"
#include "hal/msg/hal_dvl_control.hpp"

#include "hal/srv/hal_battery_control_srv.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <vector>
#include <errno.h>

#include <thread>
#include <cstring>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BspCommNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    BspCommNode(): LifecycleNode("bsp_comm_node")
    {
        this->declare_parameter<std::string>("local_ip", "192.168.137.2");
        this->declare_parameter<int>("local_port", 8113);

        this->declare_parameter<std::string>("target_ip", "192.168.137.1");
        this->declare_parameter<int>("target_port", 8114);
    }

    // ================= 生命周期 =================
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        auto qos = rclcpp::QoS(10);

        // 创建HAL节点消息订阅
        inertial_sub_     = this->create_subscription<hal::msg::HalInertialnavi>("/hal/inertialnavi",qos,std::bind(&BspCommNode::inertial_callback, this, std::placeholders::_1));
        dvl_sub_          = this->create_subscription<hal::msg::HalDvl>("/hal/dvl",qos,std::bind(&BspCommNode::dvl_callback, this, std::placeholders::_1));
        depthsensor_sub_  = this->create_subscription<hal::msg::HalDepthsensor>("/hal/depthsensor",qos,std::bind(&BspCommNode::depthsensor_callback, this, std::placeholders::_1));
        mainthruster_sub_ = this->create_subscription<hal::msg::HalMainthruster>("/hal/mainthruster",qos,std::bind(&BspCommNode::mainthruster_callback, this, std::placeholders::_1));
        auxithruster_sub_ = this->create_subscription<hal::msg::HalAuxithruster>("/hal/auxithruster",qos,std::bind(&BspCommNode::auxithruster_callback, this, std::placeholders::_1));
        battery_sub_      = this->create_subscription<hal::msg::HalBattery>("/hal/battery",qos,std::bind(&BspCommNode::battery_callback, this, std::placeholders::_1));
        tailservo_sub_    = this->create_subscription<hal::msg::HalTailservo>("/hal/tailservo",qos,std::bind(&BspCommNode::tailservo_callback, this, std::placeholders::_1));
        armmotor_sub_     = this->create_subscription<hal::msg::HalArmmotor>("/hal/armmotor",qos,std::bind(&BspCommNode::armmotor_callback, this, std::placeholders::_1));
        antenna_sub_      = this->create_subscription<hal::msg::HalAntenna>("/hal/antenna",qos,std::bind(&BspCommNode::antenna_callback, this, std::placeholders::_1));
        
        // color_image_sub_  = this->create_subscription<sensor_msgs::msg::Image>("/uvms/perception/image_raw",rclcpp::SensorDataQoS(),std::bind(&BspCommNode::color_image_callback, this, std::placeholders::_1));
        // depth_image_sub_  = this->create_subscription<sensor_msgs::msg::Image>("/uvms/perception/depth",rclcpp::SensorDataQoS(),std::bind(&BspCommNode::depth_image_callback, this, std::placeholders::_1));
        
        antenna_control_pub_    = this->create_publisher<hal::msg::HalAntennaControl>("/hal/antennacontrol", 10);
        light_control_pub_      = this->create_publisher<hal::msg::HalLightControl>("/hal/lightcontrol", 10);
        mode_control_pub_       = this->create_publisher<hal::msg::HalModeControl>("/hal/modecontrol", 10);
        remote_control_pub_     = this->create_publisher<hal::msg::HalRemoteControl>("/hal/remotecontrol", 10);
        dvl_control_pub_        = this->create_publisher<hal::msg::HalDVLControl>("/hal/dvlcontrol", 10);
        
        battery_control_client_ = this->create_client<hal::srv::HalBatteryControlSrv>("/hal/batterycontrol");
 
        local_ip_ = this->get_parameter("local_ip").as_string();
        local_port_ = this->get_parameter("local_port").as_int();

        target_ip_ = this->get_parameter("target_ip").as_string();
        target_port_ = this->get_parameter("target_port").as_int();

        // 创建UDP socket
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP socket create failed");
            return CallbackReturn::FAILURE;
        }
        
        int opt = 1;
        setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        memset(&local_addr_, 0, sizeof(local_addr_));
        local_addr_.sin_family = AF_INET;
        local_addr_.sin_port = htons(local_port_);
        
        if (inet_pton(AF_INET, local_ip_.c_str(), &local_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid local IP: %s", local_ip_.c_str());
            close(sock_);
            sock_ = -1;
            return CallbackReturn::FAILURE;
        }

        if (bind(sock_, reinterpret_cast<sockaddr*>(&local_addr_), sizeof(local_addr_)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP bind failed: %s:%d, errno=%d", local_ip_.c_str(), local_port_, errno);
            close(sock_);
            sock_ = -1;
            return CallbackReturn::FAILURE;
        }
        
        memset(&target_addr_, 0, sizeof(target_addr_));
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(target_port_);

        if (inet_pton(AF_INET, target_ip_.c_str(), &target_addr_.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid target IP: %s", target_ip_.c_str()); close(sock_);
            sock_ = -1;
            return CallbackReturn::FAILURE;
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),std::bind(&BspCommNode::udp_send, this));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
{
    active_ = true;
    udp_recv_running_ = true;

        if (light_control_pub_) {light_control_pub_->on_activate();}
        if (antenna_control_pub_) {antenna_control_pub_->on_activate();}
        if (mode_control_pub_) {mode_control_pub_->on_activate();}
        if (remote_control_pub_) {remote_control_pub_->on_activate();}
        if (dvl_control_pub_) {dvl_control_pub_->on_activate();}


    udp_recv_thread_ = std::thread(&BspCommNode::udp_receive_function, this);
    return CallbackReturn::SUCCESS;
}

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        active_ = false;
        udp_recv_running_ = false;
        if (sock_ >= 0) {::shutdown(sock_, SHUT_RDWR);}
        if (udp_recv_thread_.joinable()) {udp_recv_thread_.join();}
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        inertial_sub_.reset();
        dvl_sub_.reset();
        depthsensor_sub_.reset();
        mainthruster_sub_.reset();
        auxithruster_sub_.reset();
        battery_sub_.reset();
        tailservo_sub_.reset();
        armmotor_sub_.reset();
        antenna_sub_.reset();
        color_image_sub_.reset();
        timer_.reset();
        
        udp_recv_running_ = false;
        if (sock_ >= 0) {::shutdown(sock_, SHUT_RDWR);}
        if (udp_recv_thread_.joinable()) { udp_recv_thread_.join();}

        if (sock_ >= 0) {close(sock_); sock_ = -1;}
        
        cv::destroyAllWindows();

        if (sock_ >= 0) {
            close(sock_);
            sock_ = -1;
        }

        return CallbackReturn::SUCCESS;
    }

private:

    // ================= 回调函数 =================
    void inertial_callback(const hal::msg::HalInertialnavi::SharedPtr msg)
    {
        if (!active_) return;
        inertial_data_ = *msg;
    }
    
    void dvl_callback(const hal::msg::HalDvl::SharedPtr msg)
    {
        if (!active_) return;
        dvl_data_ = *msg;
    }
    
    void depthsensor_callback(const hal::msg::HalDepthsensor::SharedPtr msg)
    {
        if (!active_) return;
        depthsensor_data_ = *msg;
    }
    
    void mainthruster_callback(const hal::msg::HalMainthruster::SharedPtr msg)
    {
        if (!active_) return;
        mainthruster_data_ = *msg;
    }
    
    void auxithruster_callback(const hal::msg::HalAuxithruster::SharedPtr msg)
    {
        if (!active_) return;
        auxithruster_data_ = *msg;
    }
    
    void battery_callback(const hal::msg::HalBattery::SharedPtr msg)
    {
        if (!active_) return;
        battery_data_ = *msg;
    }
    
    void tailservo_callback(const hal::msg::HalTailservo::SharedPtr msg)
    {
        if (!active_) return;
        tailservo_data_ = *msg;
    }
    
    void armmotor_callback(const hal::msg::HalArmmotor::SharedPtr msg)
    {
        if (!active_) return;
        armmotor_data_ = *msg;
    }
    
    void antenna_callback(const hal::msg::HalAntenna::SharedPtr msg)
    {
        if (!active_) return;
        antenna_data_ = *msg;
    }
    
    void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!active_) return;

        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

            cv::imshow("Binocular Color Image", cv_ptr->image);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception & e)
        {RCLCPP_ERROR(this->get_logger(), "cv_bridge color error: %s", e.what());}
    }

// ================= 打包函数 =================
    std::vector<uint8_t> pack_inertial(const hal::msg::HalInertialnavi & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 8*sizeof(float));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.yaw, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.pitch, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.roll, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.latitude, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.longitude, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.east_velocity, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.north_velocity, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.sky_velocity, sizeof(float));

        return buf;
    }
    
    std::vector<uint8_t> pack_dvl(const hal::msg::HalDvl & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 3*sizeof(float));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.velocity_x, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.velocity_y, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.velocity_z, sizeof(float)); p += sizeof(float);

        return buf;
    }
    
    std::vector<uint8_t> pack_depthsensor(const hal::msg::HalDepthsensor & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 2*sizeof(float) + 2*sizeof(uint16_t) + sizeof(float));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.depth_1, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.temp_1, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.depth_2, sizeof(float)); p += sizeof(float);
        memcpy(p, &msg.temp_2, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.depth_avg, sizeof(float));

        return buf;
    }
    
    std::vector<uint8_t> pack_mainthruster(const hal::msg::HalMainthruster & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 3*sizeof(int16_t) + sizeof(uint8_t));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.rpm, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.current, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.voltage, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.fault_status, sizeof(uint8_t)); p += sizeof(uint8_t);

        return buf;
    }
    
    std::vector<uint8_t> pack_auxithruster(const hal::msg::HalAuxithruster & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 6 * sizeof(int16_t) + 6 * sizeof(int16_t) + 6 * sizeof(uint16_t) + 6 * sizeof(uint8_t) + 6 * sizeof(uint8_t));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, msg.rpm.data(), 6 * sizeof(int16_t)); p += 6 * sizeof(int16_t);
        memcpy(p, msg.current.data(), 6 * sizeof(int16_t)); p += 6 * sizeof(int16_t);
        memcpy(p, msg.voltage.data(), 6 * sizeof(uint16_t)); p += 6 * sizeof(uint16_t);
        memcpy(p, msg.esc_status.data(), 6 * sizeof(uint8_t)); p += 6 * sizeof(uint8_t);
        memcpy(p, msg.fault_status.data(), 6 * sizeof(uint8_t)); p += 6 * sizeof(uint8_t);

        return buf;
    }
    
    std::vector<uint8_t> pack_battery(const hal::msg::HalBattery & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 2*sizeof(uint8_t) + 2*sizeof(uint16_t) + 2*sizeof(int16_t) + 2*sizeof(uint16_t) + 2*sizeof(uint16_t) + 2*sizeof(uint16_t) + 2*sizeof(uint16_t) + 3*sizeof(uint8_t));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.battery_status_48v, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.battery_status_72v, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.battery_voltage_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.battery_voltage_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.battery_current_48v, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.battery_current_72v, sizeof(int16_t)); p += sizeof(int16_t);
        memcpy(p, &msg.cycle_count_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.cycle_count_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.battery_temperature_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.battery_temperature_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.remain_capacity_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.remain_capacity_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.total_capacity_48v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.total_capacity_72v, sizeof(uint16_t)); p += sizeof(uint16_t);
        memcpy(p, &msg.switch_state_12v, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.switch_state_24v, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.switch_state_72v, sizeof(uint8_t));
        
        return buf;
    }
    
    std::vector<uint8_t> pack_tailservo(const hal::msg::HalTailservo & msg)
    {
        std::vector<uint8_t> buf( sizeof(int64_t) + 4 * sizeof(float));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, msg.position.data(), 4 * sizeof(float)); p += 4 * sizeof(float);

        return buf;
    }
    
    std::vector<uint8_t> pack_armmotor(const hal::msg::HalArmmotor & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 10 * sizeof(int16_t) + 10 * sizeof(int16_t) + 10 * sizeof(int16_t) + 10 * sizeof(uint16_t) + 10 * sizeof(uint8_t));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, msg.motor_current.data(), 10 * sizeof(int16_t)); p += 10 * sizeof(int16_t);
        memcpy(p, msg.motor_speed.data(), 10 * sizeof(int16_t)); p += 10 * sizeof(int16_t);
        memcpy(p, msg.motor_position.data(), 10 * sizeof(int16_t)); p += 10 * sizeof(int16_t);
        memcpy(p, msg.motor_temp.data(), 10 * sizeof(uint16_t)); p += 10 * sizeof(uint16_t);
        memcpy(p, msg.motor_error.data(), 10 * sizeof(uint8_t)); p += 10 * sizeof(uint8_t);

        return buf;
    }
    
    std::vector<uint8_t> pack_antenna(const hal::msg::HalAntenna & msg)
    {
        std::vector<uint8_t> buf(sizeof(int64_t) + 2*sizeof(uint8_t) + sizeof(double));

        uint8_t* p = buf.data();

        memcpy(p, &msg.timestamp, sizeof(int64_t)); p += sizeof(int64_t);
        memcpy(p, &msg.brake_status, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.running_status, sizeof(uint8_t)); p += sizeof(uint8_t);
        memcpy(p, &msg.total_angle, sizeof(double));

        return buf;
    }


// ================= 打印函数 =================
    void print_inertial(const hal::msg::HalInertialnavi & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[Inertial]\n""timestamp: %ld | yaw: %.2f | pitch: %.2f | roll: %.2f\n""latitude: %.2f | longitude: %.2f\n""east_velocity: %.2f | north_velocity: %.2f | sky_velocity: %.2f",
        msg.timestamp,
        msg.yaw,
        msg.pitch,
        msg.roll,
        msg.latitude,
        msg.longitude,
        msg.east_velocity,
        msg.north_velocity,
        msg.sky_velocity
    );
    }
    
    void print_dvl(const hal::msg::HalDvl & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[Dvl]\n" "timestamp: %ld | velocity_x: %.2f | velocity_y: %.2f | velocity_z: %.2f",
        msg.timestamp,
        msg.velocity_x,
        msg.velocity_y,
        msg.velocity_z
    );
    }
    
    void print_depthsensor(const hal::msg::HalDepthsensor & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[Depthsensor]\n" "timestamp: %ld\n" "depth_1: %.3f m | temp_1: %u\n" "depth_2: %.3f m | temp_2: %u\n" "depth_avg: %.3f m",
        msg.timestamp,
        msg.depth_1,
        msg.temp_1,
        msg.depth_2,
        msg.temp_2,
        msg.depth_avg
    );
    }
    
    void print_mainthruster(const hal::msg::HalMainthruster & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[MainThruster]\n" "timestamp: %ld\n" "rpm: %d\n" "current: %d\n" "voltage: %d\n" "fault_status: %u",
        msg.timestamp,
        msg.rpm,
        msg.current,
        msg.voltage,
        msg.fault_status
    );
    }
    
    void print_auxithruster(const hal::msg::HalAuxithruster & msg)
    {
    RCLCPP_INFO(this->get_logger(),"timestamp: %ld", msg.timestamp);

    for (size_t i = 0; i < 6; ++i)
    {
    RCLCPP_INFO(this->get_logger(), "[Thruster %zu] rpm:%d | current:%d | voltage:%u | esc:%u | fault:%u", i,msg.rpm[i],msg.current[i],msg.voltage[i],msg.esc_status[i],msg.fault_status[i]);
    }
    }
    
    void print_battery(const hal::msg::HalBattery & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[Battery]\n" "ts: %ld\n" "status: 48V=%u | 72V=%u\n" "voltage: 48V=%.1fV | 72V=%.1fV\n" "current: 48V=%.1fA | 72V=%.1fA\n" "cycle: 48V=%u | 72V=%u\n" "temp: 48V=%uC | 72V=%uC\n" "remain: 48V=%.1fAh | 72V=%.1fAh\n"
                                    "total: 48V=%.1fAh | 72V=%.1fAh\n" "switch: 12V=%u | 24V=%u | 72V=%u",
        msg.timestamp,
        msg.battery_status_48v,
        msg.battery_status_72v,
        msg.battery_voltage_48v * 0.1,
        msg.battery_voltage_72v * 0.1,
        msg.battery_current_48v * 0.1,
        msg.battery_current_72v * 0.1,
        msg.cycle_count_48v,
        msg.cycle_count_72v,
        msg.battery_temperature_48v,
        msg.battery_temperature_72v,
        msg.remain_capacity_48v * 0.1,
        msg.remain_capacity_72v * 0.1,
        msg.total_capacity_48v * 0.1,
        msg.total_capacity_72v * 0.1,
        msg.switch_state_12v,
        msg.switch_state_24v,
        msg.switch_state_72v
    );
    }
    
    void print_tailservo(const hal::msg::HalTailservo & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[TailServo] timestamp: %ld", msg.timestamp);

    for (size_t i = 0; i < 4; ++i)
    {
        RCLCPP_INFO( this->get_logger(), "[TailServo %zu] position: %.2f", i, msg.position[i]);
    }
    }
    
    void print_armmotor(const hal::msg::HalArmmotor & msg)
    {
    RCLCPP_INFO(this->get_logger(), "timestamp: %ld", msg.timestamp);

    for (size_t i = 0; i < 10; ++i)
    {
        RCLCPP_INFO(this->get_logger(), "[Motor %zu] current:%d | speed:%d | position:%d | temp:%u | error:%u", i,
            msg.motor_current[i],
            msg.motor_speed[i],
            msg.motor_position[i],
            msg.motor_temp[i],
            msg.motor_error[i]
        );
    }
    }
    
    void print_antenna(const hal::msg::HalAntenna & msg)
    {
    RCLCPP_INFO(this->get_logger(), "[Antenna]\n" "timestamp: %ld\n" "brake_status: %u\n" "run_status: %u\n" "total_angle: %.3f deg",
        msg.timestamp,
        msg.brake_status,
        msg.running_status,
        msg.total_angle
    );
    }
    
    std::vector<uint8_t> build_packet(uint8_t msg_id, const std::vector<uint8_t>& payload)
    {
        // 数据长度
        uint16_t payload_len = static_cast<uint16_t>(payload.size());
        std::vector<uint8_t> packet(2 + 1 + 2 + payload_len);

        uint8_t* p = packet.data();

        *p++ = 0x55;
        *p++ = 0xAA;

        *p++ = msg_id;

        *p++ = static_cast<uint8_t>((payload_len >> 8) & 0xFF);
        *p++ = static_cast<uint8_t>(payload_len & 0xFF);

        if (payload_len > 0)
        {
            memcpy(p, payload.data(), payload_len);
        }

        return packet;
    }  
    
    // ================= UDP发送 =================
    void udp_send()
    {
        if (!active_) return;

        static int count = 0;
        bool do_print = (++count % 1000 == 0);

        // ---------- Inertialnavi ----------
        if (inertial_data_.has_value())
        {
            const auto & msg = inertial_data_.value();
            if (do_print)
            {
                print_inertial(msg);
            }

            auto payload = pack_inertial(msg);
            auto packet = build_packet(0x01, payload);

            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        } 

    // ---------- DVL ----------
        if (dvl_data_.has_value())
        {
            const auto & msg = dvl_data_.value();
            if (do_print)
            {
                print_dvl(msg);
            }

            auto payload = pack_dvl(msg);
            auto packet = build_packet(0x02, payload);

            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }

    // ---------- Depthsensor ----------
        if (depthsensor_data_.has_value())
        {
            const auto & msg = depthsensor_data_.value();
            if (do_print)
            {
                print_depthsensor(msg);
            }

            auto payload = pack_depthsensor(msg);
            auto packet = build_packet(0x03, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Mainthruster ----------
        if (mainthruster_data_.has_value())
        {
            const auto & msg = mainthruster_data_.value();
            if (do_print)
            {
                print_mainthruster(msg);
            }

            auto payload = pack_mainthruster(msg);
            auto packet = build_packet(0x04, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Auxithruster ----------
        if (auxithruster_data_.has_value())
        {
            const auto & msg = auxithruster_data_.value();
            if (do_print)
            {
                print_auxithruster(msg);
            }

            auto payload = pack_auxithruster(msg);
            auto packet = build_packet(0x05, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Battery ----------
        if (battery_data_.has_value())
        {
            const auto & msg = battery_data_.value();
            if (do_print)
            {
                print_battery(msg);
            }

            auto payload = pack_battery(msg);
            auto packet = build_packet(0x06, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Tailservo ----------
        if (tailservo_data_.has_value())
        {
            const auto & msg = tailservo_data_.value();
            if (do_print)
            {
                print_tailservo(msg);
            }

            auto payload = pack_tailservo(msg);
            auto packet = build_packet(0x07, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Armmotor ----------
        if (armmotor_data_.has_value())
        {
            const auto & msg = armmotor_data_.value();
            if (do_print)
            {
                print_armmotor(msg);
            }

            auto payload = pack_armmotor(msg);
            auto packet = build_packet(0x09, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
    // ---------- Antenna ----------
        if (antenna_data_.has_value())
        {
            const auto & msg = antenna_data_.value();
            if (do_print)
            {
                print_antenna(msg);
            }

            auto payload = pack_antenna(msg);
            auto packet = build_packet(0x12, payload);
            
            sendto(sock_, packet.data(), packet.size(), 0, reinterpret_cast<struct sockaddr*>(&target_addr_), sizeof(target_addr_));
        }
        
        
    }
    
    // ================= UDP接收 =================
    void udp_receive_function()
    {
        RCLCPP_INFO(this->get_logger(), "UDP receive thread started, listening on %s:%d", local_ip_.c_str(), local_port_);

        uint8_t buffer[2048];

        while (udp_recv_running_) {
            sockaddr_in sender_addr{};
            socklen_t sender_len = sizeof(sender_addr);

            ssize_t recv_len = recvfrom(sock_, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr*>(&sender_addr), &sender_len);

            if (recv_len <= 0) {
                if (udp_recv_running_) {
                    RCLCPP_WARN(this->get_logger(), "UDP recvfrom failed");
                }
                continue;
            }

            char sender_ip[INET_ADDRSTRLEN] = {0};
            inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, sizeof(sender_ip));
            int sender_port = ntohs(sender_addr.sin_port);

            std::string hex_str;
            char tmp[8];

            for (ssize_t i = 0; i < recv_len; ++i) {
                snprintf(tmp, sizeof(tmp), "%02X ", buffer[i]);
                hex_str += tmp;
            }

            std::vector<uint8_t> frame(buffer, buffer + recv_len);
            handle_command_frame(frame);
        }

        RCLCPP_INFO(this->get_logger(), "UDP receive thread stopped");
    }

private:
    rclcpp::Subscription<hal::msg::HalInertialnavi>::SharedPtr inertial_sub_;
    rclcpp::Subscription<hal::msg::HalDvl>::SharedPtr dvl_sub_;
    rclcpp::Subscription<hal::msg::HalDepthsensor>::SharedPtr depthsensor_sub_;
    rclcpp::Subscription<hal::msg::HalMainthruster>::SharedPtr mainthruster_sub_;
    rclcpp::Subscription<hal::msg::HalAuxithruster>::SharedPtr auxithruster_sub_;
    rclcpp::Subscription<hal::msg::HalBattery>::SharedPtr battery_sub_;
    rclcpp::Subscription<hal::msg::HalTailservo>::SharedPtr tailservo_sub_;
    rclcpp::Subscription<hal::msg::HalArmmotor>::SharedPtr armmotor_sub_;
    rclcpp::Subscription<hal::msg::HalAntenna>::SharedPtr antenna_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalAntennaControl>::SharedPtr antenna_control_pub_;
    rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalLightControl>::SharedPtr light_control_pub_;
    rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalModeControl>::SharedPtr mode_control_pub_;
    rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalRemoteControl>::SharedPtr remote_control_pub_;
    rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalDVLControl>::SharedPtr dvl_control_pub_;
    rclcpp::Client<hal::srv::HalBatteryControlSrv>::SharedPtr battery_control_client_;

    std::optional<hal::msg::HalInertialnavi> inertial_data_;
    std::optional<hal::msg::HalDvl> dvl_data_;
    std::optional<hal::msg::HalDepthsensor> depthsensor_data_;
    std::optional<hal::msg::HalMainthruster> mainthruster_data_;
    std::optional<hal::msg::HalAuxithruster> auxithruster_data_;
    std::optional<hal::msg::HalBattery> battery_data_;
    std::optional<hal::msg::HalTailservo> tailservo_data_;
    std::optional<hal::msg::HalArmmotor> armmotor_data_;
    std::optional<hal::msg::HalAntenna> antenna_data_;

    int sock_{-1};
    std::string local_ip_;
    int local_port_{8113};
    std::string target_ip_;
    int target_port_{8114};
    struct sockaddr_in local_addr_;
    struct sockaddr_in target_addr_;
    std::thread udp_recv_thread_;
    std::atomic<bool> udp_recv_running_{false};

    bool active_{false};
    
    void handle_command_frame(const std::vector<uint8_t>& frame)
    {
        if (frame.size() < 5) {RCLCPP_WARN(this->get_logger(), "Command frame too short"); return;} 

        if (frame[0] != 0x55 || frame[1] != 0xAA) {RCLCPP_WARN(this->get_logger(), "Invalid command header"); return;}

        uint8_t msg_id = frame[2];
        uint16_t payload_len = (static_cast<uint16_t>(frame[3]) << 8) | static_cast<uint16_t>(frame[4]);

        if (frame.size() != static_cast<size_t>(5 + payload_len)) {RCLCPP_WARN(this->get_logger(), "Invalid command length"); return;}

        std::vector<uint8_t> payload(frame.begin() + 5, frame.end());
        RCLCPP_INFO(this->get_logger(), "Received command: msg_id=0x%02X, payload_len=%d", msg_id, payload_len);

        dispatch_command(msg_id, payload);
    }
    
    void dispatch_command(uint8_t msg_id, const std::vector<uint8_t>& payload)
    {
        switch(msg_id)
        {
             // 灯光控制
            case 0x30:{light_control(payload); break;}
            
            // 天线控制
            case 0x31:
            {antenna_control(payload); break;}
            
            // 电池控制
            case 0x35:
            {battery_control(payload); break;}

            // 模式控制
            case 0x41:
            {mode_control(payload); break;}
            
            // 遥控控制
            case 0x42:
            {remote_control(payload); break;}

            // DVL控制
            case 0x44:
            {dvl_control(payload); break;}

            default:
            {RCLCPP_WARN(this->get_logger(), "Unknown command id: 0x%02X", msg_id); break;}
        }
    }
    
    // 灯光控制
    
    void light_control(const std::vector<uint8_t>& payload)
    {
        if(payload.size() != 1) {RCLCPP_WARN(this->get_logger(), "Light command payload length error: %ld", payload.size()); return;}

        uint8_t light_coeff = payload[0];

        RCLCPP_INFO(this->get_logger(), "Light command received: brightness=%d", light_coeff);
        hal::msg::HalLightControl msg;
        msg.light_coeff = light_coeff;

        light_control_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Publish light control command");
    }
    
    // 天线控制 
    
    void antenna_control(const std::vector<uint8_t>& payload)
    {
        if(payload.size() != 5) {RCLCPP_WARN(this->get_logger(), "Antenna command payload length error: %ld", payload.size()); return;}

        uint8_t cmd_type;
        float target_coeff;
        const uint8_t* p = payload.data();

        // 解析运行状态
        memcpy(&cmd_type, p, sizeof(uint8_t));
        p += sizeof(uint8_t);
        // 解析位置系数
        memcpy(&target_coeff, p, sizeof(float));

        RCLCPP_INFO(this->get_logger(), "Antenna command: cmd_type=%d, target_coeff=%.3f", cmd_type, target_coeff);
        // 构造ROS消息
        hal::msg::HalAntennaControl msg;
        msg.cmd_type = cmd_type;
        msg.target_coeff = target_coeff;
        // 发布给HAL天线节点
        antenna_control_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published /hal/antenna_control");
    }

     // 模式控制 
    void mode_control(const std::vector<uint8_t>& payload)
    {
        if(payload.size() != 1) {RCLCPP_WARN(this->get_logger(), "Mode command payload length error: %ld", payload.size()); return;}

        uint8_t mode_cmd = payload[0];

        auto msg = hal::msg::HalModeControl();
        msg.modecontrol_cmd = mode_cmd;
        mode_control_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),"Publish mode control command: %d", mode_cmd);
    }
    
    // 遥控控制 
    void remote_control(const std::vector<uint8_t>& payload)
    {
        if(payload.size() != 24) {RCLCPP_WARN(this->get_logger(), "Remote control payload length error: %ld", payload.size()); return;}

        float tunnel1;
        float tunnel2;
        float tunnel3;
        float tunnel4;
        float tunnel5;
        float tunnel6;

        memcpy(&tunnel1, payload.data(), sizeof(float));
        memcpy(&tunnel2, payload.data()+4, sizeof(float));
        memcpy(&tunnel3, payload.data()+8, sizeof(float));
        memcpy(&tunnel4, payload.data()+12, sizeof(float));
        memcpy(&tunnel5, payload.data()+16, sizeof(float));
        memcpy(&tunnel6, payload.data()+20, sizeof(float));

        auto msg = hal::msg::HalRemoteControl();

        msg.tunnel1_para = tunnel1;
        msg.tunnel2_para = tunnel2;
        msg.tunnel3_para = tunnel3;
        msg.tunnel4_para = tunnel4;
        msg.tunnel5_para = tunnel5;
        msg.tunnel6_para = tunnel6;

        remote_control_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Remote control: %.3f %.3f %.3f %.3f %.3f %.3f", tunnel1, tunnel2, tunnel3, tunnel4, tunnel5, tunnel6);
    }
    
    // DVL控制 
    void dvl_control(const std::vector<uint8_t>& payload)
    {
        if(payload.size() != 1) {RCLCPP_WARN(this->get_logger(), "DVL command payload length error: %ld", payload.size()); return;}

        uint8_t dvl_cmd = payload[0];

        auto msg = hal::msg::HalDVLControl();
        msg.dvlcontrol_cmd = dvl_cmd;
        dvl_control_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),"Publish DVL control command: %d", dvl_cmd);
    }
    
    void battery_control(const std::vector<uint8_t>& payload)
    {
        if (!battery_control_client_->service_is_ready()) {RCLCPP_WARN(this->get_logger(), "/hal/batterycontrol service not ready"); return;}
        if(payload.size() != 1) {RCLCPP_WARN(this->get_logger(), "Battery command payload length error: %ld", payload.size()); return;}
        
        uint8_t cmd = payload[0];
        RCLCPP_INFO(this->get_logger(), "Battery command received: cmd=0x%02X", cmd);

        auto request = std::make_shared<hal::srv::HalBatteryControlSrv::Request>();
        request->command = cmd;

        battery_control_client_->async_send_request(request, [this, cmd](rclcpp::Client<hal::srv::HalBatteryControlSrv>::SharedFuture future)
            {
                try {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "Battery service response: cmd=0x%02X, success=%d, message=%s", cmd, response->success, response->message.c_str());
                    }
                catch (const std::exception& e) {RCLCPP_ERROR(this->get_logger(), "Battery service call failed: %s", e.what());}
            }
        );
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BspCommNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}