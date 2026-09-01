#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hal/msg/hal_battery.hpp"
#include "hal/srv/hal_battery_control_srv.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class BspStartupManagerNode : public rclcpp::Node
{
public:
  BspStartupManagerNode()
  : Node("bsp_startupmanager_node")
  {
    declare_parameter<bool>("autostart", true);
    declare_parameter<bool>("require_power_feedback", true);
    declare_parameter<double>("service_timeout_s", 5.0);
    declare_parameter<double>("power_feedback_timeout_s", 8.0);
    declare_parameter<double>("min_48v_voltage", 36.0);
    declare_parameter<int>("power_stabilize_ms", 1200);
    declare_parameter<int>("lifecycle_bringup_max_attempts", 3);
    declare_parameter<int>("lifecycle_bringup_retry_delay_ms", 1000);

    declare_parameter<std::vector<std::string>>(
      "core_lifecycle_nodes", {"/bsp_comm_node"});
    declare_parameter<std::vector<std::string>>(
      "rail_12v_lifecycle_nodes",
      {"/hal_inertialnavi_node", "/hal_light_sw_pwm_node", "/hal_binocamera_node"});
    declare_parameter<std::vector<std::string>>(
      "rail_24v_lifecycle_nodes",
      {"/hal_dvl_node", "/hal_depthsensor_node", "/hal_cabinmotor_node",
        "/hal_antenna_lifecycle_node"});
    declare_parameter<std::vector<std::string>>(
      "bsp_lifecycle_nodes",
      {"/bsp_remotecontrol_node", "/bsp_motioncontrol_node"});
    declare_parameter<std::vector<std::string>>(
      "rail_72v_lifecycle_nodes", {"/hal_thruster_node"});

    autostart_ = get_parameter("autostart").as_bool();
    require_power_feedback_ = get_parameter("require_power_feedback").as_bool();
    service_timeout_ = seconds_parameter("service_timeout_s");
    power_feedback_timeout_ = seconds_parameter("power_feedback_timeout_s");
    min_48v_voltage_ = get_parameter("min_48v_voltage").as_double();
    power_stabilize_delay_ =
      std::chrono::milliseconds(get_parameter("power_stabilize_ms").as_int());
    lifecycle_bringup_max_attempts_ =
      get_parameter("lifecycle_bringup_max_attempts").as_int();
    if (lifecycle_bringup_max_attempts_ < 1) {
      lifecycle_bringup_max_attempts_ = 1;
    }
    const auto retry_delay_ms = get_parameter("lifecycle_bringup_retry_delay_ms").as_int();
    lifecycle_bringup_retry_delay_ =
      std::chrono::milliseconds(retry_delay_ms < 0 ? 0 : retry_delay_ms);

    core_nodes_ = get_parameter("core_lifecycle_nodes").as_string_array();
    rail_12v_nodes_ = get_parameter("rail_12v_lifecycle_nodes").as_string_array();
    rail_24v_nodes_ = get_parameter("rail_24v_lifecycle_nodes").as_string_array();
    bsp_nodes_ = get_parameter("bsp_lifecycle_nodes").as_string_array();
    rail_72v_nodes_ = get_parameter("rail_72v_lifecycle_nodes").as_string_array();

    battery_sub_ = create_subscription<hal::msg::HalBattery>(
      "/hal/battery",
      rclcpp::QoS(10).reliable(),
      std::bind(&BspStartupManagerNode::battery_callback, this, std::placeholders::_1));

    battery_client_ = create_client<hal::srv::HalBatteryControlSrv>("/hal/batterycontrol");

    if (autostart_) {
      startup_timer_ = create_wall_timer(500ms, [this]() {
        startup_timer_->cancel();
        start_worker();
      });
    }
  }

  ~BspStartupManagerNode() override
  {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      shutting_down_ = true;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  enum class Rail : uint8_t { V12, V24, V72 };

  std::chrono::milliseconds seconds_parameter(const std::string & name) const
  {
    const auto seconds = get_parameter(name).as_double();
    return std::chrono::milliseconds(static_cast<int>(seconds * 1000.0));
  }

  static std::string normalize_node_name(const std::string & name)
  {
    if (name.empty()) {
      return name;
    }
    return name.front() == '/' ? name : "/" + name;
  }

  void battery_callback(const hal::msg::HalBattery::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    latest_battery_ = *msg;
  }

  void start_worker()
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_started_) {
      return;
    }
    worker_started_ = true;
    worker_ = std::thread(&BspStartupManagerNode::run_startup_sequence, this);
  }

  void run_startup_sequence()
  {
    RCLCPP_INFO(get_logger(), "UVMS startup sequence started.");

    if (!bringup_lifecycle_node_with_retry("/hal_battery_node")) {
      fail("hal_battery_node bringup failed after retries");
      return;
    }

    if (!wait_for_48v_ready()) {
      fail("48V default power rail is not ready");
      return;
    }

    if (!bringup_group("core", core_nodes_)) {
      return;
    }

    // Main thruster power-on requires the 72V rail before the 12V rail.
    if (!enable_rail(Rail::V72)) {
      return;
    }

    if (!enable_rail(Rail::V12) || !bringup_group("12V", rail_12v_nodes_)) {
      return;
    }

    if (!enable_rail(Rail::V24) || !bringup_group("24V", rail_24v_nodes_)) {
      return;
    }

    // Keep thruster lifecycle activation until all required low-voltage rails are ready.
    if (!bringup_group("72V", rail_72v_nodes_)) {
      return;
    }

    if (!bringup_group("BSP", bsp_nodes_)) {
      return;
    }

    RCLCPP_INFO(get_logger(), "UVMS startup sequence completed.");
  }

  void fail(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "UVMS startup sequence stopped: %s", reason.c_str());
  }

  bool bringup_group(const std::string & group_name, const std::vector<std::string> & nodes)
  {
    RCLCPP_INFO(get_logger(), "Bringing up %s lifecycle group.", group_name.c_str());
    std::size_t skipped_count = 0;
    for (const auto & node : nodes) {
      if (!bringup_lifecycle_node_with_retry(node)) {
        ++skipped_count;
        RCLCPP_WARN(
          get_logger(), "Skipping %s lifecycle node after %d failed attempt(s): %s",
          group_name.c_str(), lifecycle_bringup_max_attempts_, node.c_str());
      }
    }
    if (skipped_count > 0) {
      RCLCPP_WARN(
        get_logger(), "%s lifecycle group completed with %zu skipped node(s).",
        group_name.c_str(), skipped_count);
    }
    return true;
  }

  bool bringup_lifecycle_node_with_retry(const std::string & node_name)
  {
    for (int attempt = 1; rclcpp::ok() && attempt <= lifecycle_bringup_max_attempts_; ++attempt) {
      RCLCPP_INFO(
        get_logger(), "Lifecycle bringup attempt %d/%d: %s",
        attempt, lifecycle_bringup_max_attempts_, normalize_node_name(node_name).c_str());
      if (bringup_lifecycle_node(node_name)) {
        return true;
      }
      if (attempt < lifecycle_bringup_max_attempts_) {
        std::this_thread::sleep_for(lifecycle_bringup_retry_delay_);
      }
    }
    return false;
  }

  bool bringup_lifecycle_node(const std::string & node_name)
  {
    const auto node = normalize_node_name(node_name);
    const auto state = get_lifecycle_state(node);
    if (!state.has_value()) {
      RCLCPP_ERROR(get_logger(), "Lifecycle state service unavailable: %s", node.c_str());
      return false;
    }

    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(get_logger(), "%s already active.", node.c_str());
      return true;
    }

    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      if (!change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
        return false;
      }
    }

    const auto after_configure = get_lifecycle_state(node);
    if (!after_configure.has_value()) {
      return false;
    }

    if (*after_configure == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      return change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }

    if (*after_configure == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return true;
    }

    RCLCPP_ERROR(
      get_logger(), "%s is in unsupported lifecycle state %u.",
      node.c_str(), static_cast<unsigned>(*after_configure));
    return false;
  }

  std::optional<uint8_t> get_lifecycle_state(const std::string & node)
  {
    auto client = get_state_clients_[node];
    if (!client) {
      client = create_client<lifecycle_msgs::srv::GetState>(node + "/get_state");
      get_state_clients_[node] = client;
    }

    if (!client->wait_for_service(service_timeout_)) {
      return std::nullopt;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
      return std::nullopt;
    }

    return future.get()->current_state.id;
  }

  bool change_lifecycle_state(const std::string & node, uint8_t transition_id)
  {
    auto client = change_state_clients_[node];
    if (!client) {
      client = create_client<lifecycle_msgs::srv::ChangeState>(node + "/change_state");
      change_state_clients_[node] = client;
    }

    if (!client->wait_for_service(service_timeout_)) {
      RCLCPP_ERROR(get_logger(), "Lifecycle change_state service unavailable: %s", node.c_str());
      return false;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    auto future = client->async_send_request(request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Lifecycle transition timeout: %s", node.c_str());
      return false;
    }

    const bool success = future.get()->success;
    if (!success) {
      RCLCPP_ERROR(
        get_logger(), "Lifecycle transition %u failed: %s",
        static_cast<unsigned>(transition_id), node.c_str());
    }
    return success;
  }

  bool enable_rail(Rail rail)
  {
    const uint8_t command = rail_on_command(rail);
    const char * name = rail_name(rail);

    if (!battery_client_->wait_for_service(service_timeout_)) {
      fail("/hal/batterycontrol service unavailable");
      return false;
    }

    auto request = std::make_shared<hal::srv::HalBatteryControlSrv::Request>();
    request->command = command;
    auto future = battery_client_->async_send_request(request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
      fail(std::string(name) + " power control timeout");
      return false;
    }

    auto response = future.get();
    if (!response->success) {
      fail(std::string(name) + " power control failed: " + response->message);
      return false;
    }

    std::this_thread::sleep_for(power_stabilize_delay_);
    if (!wait_for_rail_feedback(rail)) {
      fail(std::string(name) + " power feedback not ready");
      return false;
    }

    RCLCPP_INFO(get_logger(), "%s power rail is ready.", name);
    return true;
  }

  uint8_t rail_on_command(Rail rail) const
  {
    switch (rail) {
      case Rail::V12:
        return 1;
      case Rail::V24:
        return 3;
      case Rail::V72:
        return 5;
    }
    return 0;
  }

  const char * rail_name(Rail rail) const
  {
    switch (rail) {
      case Rail::V12:
        return "12V";
      case Rail::V24:
        return "24V";
      case Rail::V72:
        return "72V";
    }
    return "unknown";
  }

  bool wait_for_48v_ready()
  {
    if (!require_power_feedback_) {
      return true;
    }

    const auto deadline = std::chrono::steady_clock::now() + power_feedback_timeout_;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto msg = latest_battery_msg();
      if (msg.has_value() && msg->battery_voltage_48v * 0.1 >= min_48v_voltage_) {
        RCLCPP_INFO(
          get_logger(), "48V rail feedback ready: %.1f V.",
          msg->battery_voltage_48v * 0.1);
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  bool wait_for_rail_feedback(Rail rail)
  {
    if (!require_power_feedback_) {
      return true;
    }

    const auto deadline = std::chrono::steady_clock::now() + power_feedback_timeout_;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto msg = latest_battery_msg();
      if (msg.has_value() && rail_switch_on(*msg, rail)) {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  std::optional<hal::msg::HalBattery> latest_battery_msg() const
  {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    return latest_battery_;
  }

  bool rail_switch_on(const hal::msg::HalBattery & msg, Rail rail) const
  {
    switch (rail) {
      case Rail::V12:
        return msg.switch_state_12v != 0;
      case Rail::V24:
        return msg.switch_state_24v != 0;
      case Rail::V72:
        return msg.switch_state_72v != 0;
    }
    return false;
  }

  bool autostart_{true};
  bool require_power_feedback_{true};
  bool worker_started_{false};
  bool shutting_down_{false};
  double min_48v_voltage_{36.0};
  std::chrono::milliseconds service_timeout_{5000};
  std::chrono::milliseconds power_feedback_timeout_{8000};
  std::chrono::milliseconds power_stabilize_delay_{1200};
  int lifecycle_bringup_max_attempts_{3};
  std::chrono::milliseconds lifecycle_bringup_retry_delay_{1000};

  std::vector<std::string> core_nodes_;
  std::vector<std::string> rail_12v_nodes_;
  std::vector<std::string> rail_24v_nodes_;
  std::vector<std::string> bsp_nodes_;
  std::vector<std::string> rail_72v_nodes_;

  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::Subscription<hal::msg::HalBattery>::SharedPtr battery_sub_;
  rclcpp::Client<hal::srv::HalBatteryControlSrv>::SharedPtr battery_client_;

  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>
    get_state_clients_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr>
    change_state_clients_;

  mutable std::mutex battery_mutex_;
  std::optional<hal::msg::HalBattery> latest_battery_;

  std::mutex worker_mutex_;
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BspStartupManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
