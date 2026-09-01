#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <vector>
#include <string>

class SysMonitorNode : public rclcpp::Node
{
public:
    SysMonitorNode() : Node("bsp_monitor_node")
    {
        target_nodes_ = {
            "hal_thruster_node", "hal_servo_node", "hal_antennacontrol_node",
            "hal_lightcontrol_node", "hal_inertialnavi_node", "hal_dvl_node",
            "hal_depthsenor_node", "hal_battery_node", "hal_monocamera_node",
            "hal_binocamera_node", "hal_arm_node", "hal_cabin_node"
        };

        // 初始化成员变量，大小12，全填0（0代表未建立/离线）
        current_states_.resize(target_nodes_.size(), 0);

        for (const auto& node_name : target_nodes_) {
            std::string srv_name = "/" + node_name + "/get_state";
            clients_.push_back(this->create_client<lifecycle_msgs::srv::GetState>(srv_name));
        }

        state_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/system/lifecycle_states", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SysMonitorNode::poll_states, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "System Monitor Node started. Polling %zu nodes via Services.", target_nodes_.size());
    }

private:
    void poll_states()
    {
        // 1. 触发异步查询更新内部状态
        for (size_t i = 0; i < target_nodes_.size(); ++i) {
            auto client = clients_[i];
            
            if (!client->service_is_ready()) {
                current_states_[i] = 0; // 服务不在线，重置为 0
                continue;
            }

            auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
            
            // 发起异步请求，Lambda 表达式只按值捕获 this 和 i，绝对安全
            client->async_send_request(request, 
                [this, i](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        this->current_states_[i] = response->current_state.id; 
                    } catch (const std::exception &e) {
                        this->current_states_[i] = 0;
                    }
                }
            );
        }

        // 2. 将当前的已知状态打包发布
        auto msg = std_msgs::msg::UInt8MultiArray();
        msg.data = current_states_;
        state_pub_->publish(msg);
    }

    std::vector<std::string> target_nodes_;
    std::vector<uint8_t> current_states_; // 提升为类成员变量，保障生命周期
    std::vector<rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> clients_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SysMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
