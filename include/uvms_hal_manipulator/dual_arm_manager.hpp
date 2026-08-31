#ifndef UVMS_HAL_MANIPULATOR_DUAL_ARM_MANAGER_HPP
#define UVMS_HAL_MANIPULATOR_DUAL_ARM_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace uvms_hal_manipulator
{

class DualArmLifecycleManager : public rclcpp::Node
{
public:
    explicit DualArmLifecycleManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~DualArmLifecycleManager() override = default;

private:
    // 回调
    void left_fault_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void right_fault_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // 故障处理
    void handle_dual_arm_fault(const std::string &source_arm);
    void deactivate_both_arms();

    // 服务辅助
    void wait_for_service_or_warn(
        const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr &client,
        const std::string &service_name);

    void call_deactivate(
        const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr &client,
        const std::string &arm_name);

private:
    // 订阅
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_fault_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_fault_sub_;

    // 生命周期客户端
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr left_change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr right_change_state_client_;

    // 故障锁存
    bool fault_latched_{false};
};

}  // namespace uvms_hal_manipulator

#endif