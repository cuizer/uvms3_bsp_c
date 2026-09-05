#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "hal/msg/hal_battery.hpp"
#include "hal/msg/hal_lifecyclestates_control.hpp"
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
    declare_parameter<double>("min_48v_voltage", 36.0);

    // Automatically request 72 V ON first, then request 12 V ON after this delay.
    declare_parameter<bool>("auto_power_on", true);
    declare_parameter<int>("auto_12v_delay_ms", 1000);

    // A rail must remain ON for this long before its lifecycle group starts configure.
    declare_parameter<int>("power_stabilize_ms", 2000);

    // After configure, wait this long before activate.
    declare_parameter<int>("configure_to_activate_delay_ms", 5000);

    declare_parameter<int>("lifecycle_bringup_max_attempts", 3);
    declare_parameter<int>("lifecycle_bringup_retry_delay_ms", 1000);

    // These two nodes are always brought up first, before any power-dependent group.
    declare_parameter<std::vector<std::string>>(
      "initial_lifecycle_nodes", {"/hal_battery_node", "/bsp_comm_node"});

    // These nodes gate entry into power-feedback-driven bringup. bsp_comm is still
    // started in the initial group, but it must not block unrelated rail groups.
    declare_parameter<std::vector<std::string>>(
      "initial_required_lifecycle_nodes", {"/hal_battery_node"});

    // 48 V dependent software/control group.
    declare_parameter<std::vector<std::string>>(
      "rail_48v_lifecycle_nodes",
      {"/bsp_remotecontrol_node", "/bsp_motioncontrol_node"});

    declare_parameter<std::vector<std::string>>(
      "rail_12v_lifecycle_nodes",
      {"/hal_inertialnavi_node", "/hal_light_sw_pwm_node", "/hal_binocamera_node",
        "/hal_servo_node"});

    declare_parameter<std::vector<std::string>>(
      "rail_24v_lifecycle_nodes",
      {"/hal_dvl_node", "/hal_depthsensor_node", "/hal_acoustic_node",
        "/hal_cabinmotor_node", "/hal_antenna_lifecycle_node"});

    // Thruster is special: activation gate is 72 V AND 12 V.
    declare_parameter<std::vector<std::string>>(
      "thruster_lifecycle_nodes", {"/hal_thruster_node"});

    autostart_ = get_parameter("autostart").as_bool();
    require_power_feedback_ = get_parameter("require_power_feedback").as_bool();
    service_timeout_ = seconds_parameter("service_timeout_s");
    min_48v_voltage_ = get_parameter("min_48v_voltage").as_double();
    auto_power_on_ = get_parameter("auto_power_on").as_bool();

    const auto auto_12v_delay_ms = get_parameter("auto_12v_delay_ms").as_int();
    auto_12v_delay_ =
      std::chrono::milliseconds(auto_12v_delay_ms < 0 ? 0 : auto_12v_delay_ms);

    const auto power_stabilize_ms = get_parameter("power_stabilize_ms").as_int();
    power_stabilize_delay_ =
      std::chrono::milliseconds(power_stabilize_ms < 0 ? 0 : power_stabilize_ms);

    const auto configure_to_activate_delay_ms =
      get_parameter("configure_to_activate_delay_ms").as_int();
    configure_to_activate_delay_ = std::chrono::milliseconds(
      configure_to_activate_delay_ms < 0 ? 0 : configure_to_activate_delay_ms);

    lifecycle_bringup_max_attempts_ =
      get_parameter("lifecycle_bringup_max_attempts").as_int();
    if (lifecycle_bringup_max_attempts_ < 1) {
      lifecycle_bringup_max_attempts_ = 1;
    }

    const auto retry_delay_ms = get_parameter("lifecycle_bringup_retry_delay_ms").as_int();
    lifecycle_bringup_retry_delay_ =
      std::chrono::milliseconds(retry_delay_ms < 0 ? 0 : retry_delay_ms);

    initial_nodes_ = get_parameter("initial_lifecycle_nodes").as_string_array();
    initial_required_nodes_ =
      get_parameter("initial_required_lifecycle_nodes").as_string_array();
    rail_48v_nodes_ = get_parameter("rail_48v_lifecycle_nodes").as_string_array();
    rail_12v_nodes_ = get_parameter("rail_12v_lifecycle_nodes").as_string_array();
    rail_24v_nodes_ = get_parameter("rail_24v_lifecycle_nodes").as_string_array();
    thruster_nodes_ = get_parameter("thruster_lifecycle_nodes").as_string_array();

    battery_sub_ = create_subscription<hal::msg::HalBattery>(
      "/hal/battery",
      rclcpp::QoS(10).reliable(),
      std::bind(&BspStartupManagerNode::battery_callback, this, std::placeholders::_1));

    battery_control_client_ =
      create_client<hal::srv::HalBatteryControlSrv>("/hal/batterycontrol");

    lifecycle_control_sub_ =
      create_subscription<hal::msg::HalLifecyclestatesControl>(
      "/bsp/lifecyclestatescontrol",
      rclcpp::QoS(10).reliable(),
      std::bind(
        &BspStartupManagerNode::lifecycle_control_callback, this,
        std::placeholders::_1));

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
  using Clock = std::chrono::steady_clock;

  struct GroupRuntime
  {
    bool completed{false};
    std::optional<Clock::time_point> power_on_since;
    struct NodeRuntime
    {
      bool completed{false};
      std::optional<Clock::time_point> activate_not_before;
      Clock::time_point next_retry_at{Clock::time_point::min()};
    };
    std::map<std::string, NodeRuntime> nodes;
  };

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

  void lifecycle_control_callback(
    const hal::msg::HalLifecyclestatesControl::SharedPtr msg)
  {
    const auto node = lifecycle_node_name(msg->lifecycle_node);
    if (!node.has_value()) {
      RCLCPP_WARN(
        get_logger(), "Ignoring lifecycle control for reserved/unknown node id: %u",
        static_cast<unsigned>(msg->lifecycle_node));
      return;
    }

    switch (msg->states_cmd) {
      case 1:
        handle_configure_command(*node);
        break;
      case 2:
        handle_activate_command(*node);
        break;
      default:
        RCLCPP_WARN(
          get_logger(), "Ignoring unsupported lifecycle states_cmd: %u",
          static_cast<unsigned>(msg->states_cmd));
        break;
    }
  }

  static std::optional<std::string> lifecycle_node_name(uint8_t lifecycle_node)
  {
    switch (lifecycle_node) {
      case 1:
        return "/hal_inertialnavi_node";
      case 2:
        return "/hal_dvl_node";
      case 3:
        return "/hal_depthsensor_node";
      case 4:
        return "/hal_acoustic_node";
      case 5:
        return "/hal_thruster_node";
      case 6:
        return "/hal_servo_node";
      case 7:
        return "/hal_antenna_lifecycle_node";
      case 8:
        return "/hal_light_sw_pwm_node";
      default:
        return std::nullopt;
    }
  }

  void handle_configure_command(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(lifecycle_transition_mutex_);
    const auto node = normalize_node_name(node_name);
    const auto state = get_lifecycle_state(node);
    if (!state.has_value()) {
      RCLCPP_WARN(get_logger(), "Manual configure skipped; state unavailable: %s", node.c_str());
      return;
    }

    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      const auto before_configure = get_lifecycle_state(node);
      if (!before_configure.has_value()) {
        RCLCPP_WARN(
          get_logger(), "Manual configure skipped; state recheck failed: %s",
          node.c_str());
        return;
      }
      if (*before_configure != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        RCLCPP_INFO(
          get_logger(), "Manual configure skipped; node state changed to %u: %s",
          static_cast<unsigned>(*before_configure), node.c_str());
        return;
      }

      RCLCPP_INFO(get_logger(), "Manual configuring lifecycle node: %s", node.c_str());
      change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      return;
    }

    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_INFO(
        get_logger(), "Manual configure skipped; node is already configured: %s",
        node.c_str());
      return;
    }

    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(
        get_logger(), "Manual configure skipped; node is already active: %s",
        node.c_str());
      return;
    }

    RCLCPP_WARN(
      get_logger(), "Manual configure skipped; unsupported state %u: %s",
      static_cast<unsigned>(*state), node.c_str());
  }

  void handle_activate_command(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(lifecycle_transition_mutex_);
    const auto node = normalize_node_name(node_name);
    const auto state = get_lifecycle_state(node);
    if (!state.has_value()) {
      RCLCPP_WARN(get_logger(), "Manual activate skipped; state unavailable: %s", node.c_str());
      return;
    }

    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      const auto before_activate = get_lifecycle_state(node);
      if (!before_activate.has_value()) {
        RCLCPP_WARN(
          get_logger(), "Manual activate skipped; state recheck failed: %s",
          node.c_str());
        return;
      }
      if (*before_activate != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_INFO(
          get_logger(), "Manual activate skipped; node state changed to %u: %s",
          static_cast<unsigned>(*before_activate), node.c_str());
        return;
      }

      RCLCPP_INFO(get_logger(), "Manual activating lifecycle node: %s", node.c_str());
      change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      return;
    }

    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_INFO(
        get_logger(), "Manual activate skipped; node is already active: %s",
        node.c_str());
      return;
    }

    if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      RCLCPP_WARN(
        get_logger(), "Manual activate skipped; node is not configured: %s",
        node.c_str());
      return;
    }

    RCLCPP_WARN(
      get_logger(), "Manual activate skipped; unsupported state %u: %s",
      static_cast<unsigned>(*state), node.c_str());
  }

  std::optional<hal::msg::HalBattery> latest_battery_msg() const
  {
    std::lock_guard<std::mutex> lock(battery_mutex_);
    return latest_battery_;
  }

  bool send_battery_control_command(uint8_t command, const std::string & description)
  {
    if (!battery_control_client_) {
      RCLCPP_ERROR(get_logger(), "Battery control client is not initialized.");
      return false;
    }

    if (!battery_control_client_->wait_for_service(service_timeout_)) {
      RCLCPP_WARN(
        get_logger(), "Battery control service unavailable while requesting: %s",
        description.c_str());
      return false;
    }

    auto request = std::make_shared<hal::srv::HalBatteryControlSrv::Request>();
    request->command = command;

    RCLCPP_INFO(
      get_logger(), "Sending battery control command: %s, command=%u",
      description.c_str(), static_cast<unsigned>(command));

    auto future = battery_control_client_->async_send_request(request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
      RCLCPP_ERROR(
        get_logger(), "Battery control request timeout: %s", description.c_str());
      return false;
    }

    const auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(
        get_logger(), "Battery control failed: %s, response=%s",
        description.c_str(), response->message.c_str());
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Battery control accepted: %s, response=%s",
      description.c_str(), response->message.c_str());
    return true;
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

  bool stop_requested()
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    return shutting_down_;
  }

  void run_startup_sequence()
  {
    RCLCPP_INFO(get_logger(), "UVMS startup manager started.");

    // ------------------------------------------------------------------------
    // Battery + bsp_comm are always attempted first. A bsp_comm failure must not
    // block later rail groups forever; hal_battery is the required initial node
    // because it provides the power-control service and rail feedback.
    // ------------------------------------------------------------------------
    GroupRuntime initial_runtime;
    initial_runtime.power_on_since = Clock::now();

    // ------------------------------------------------------------------------
    // Power-dependent groups: no fixed rail ordering.
    // Each group has its own state machine and reacts only to its own power gate.
    // ------------------------------------------------------------------------
    GroupRuntime rail_48v_runtime;
    GroupRuntime rail_12v_runtime;
    GroupRuntime rail_24v_runtime;
    GroupRuntime thruster_runtime;

    bool power_groups_enabled = false;
    bool initial_required_ready = false;

    // Automatic power-on state:
    //   hal_battery ACTIVE -> command 5 (72 V ON)
    //   wait auto_12v_delay_ -> command 1 (12 V ON)
    bool auto_72v_command_sent = false;
    bool auto_12v_command_sent = false;
    std::optional<Clock::time_point> auto_72v_command_time;

    while (rclcpp::ok() && !stop_requested()) {
      process_group(
        "initial", initial_nodes_, true, initial_runtime,
        std::chrono::milliseconds(0));

      // Check the required initial nodes independently. This intentionally allows
      // a bsp_comm failure to not block the power-dependent HAL nodes.
      if (!initial_required_ready) {
        initial_required_ready = required_initial_nodes_active();
        if (initial_required_ready) {
          RCLCPP_INFO(get_logger(), "Required initial lifecycle nodes are ACTIVE.");
        }
      }

      if (!power_groups_enabled &&
        (initial_runtime.completed || !require_power_feedback_ || initial_required_ready))
      {
        power_groups_enabled = true;
        RCLCPP_INFO(
          get_logger(),
          "Initial lifecycle gate is ready. Entering power-feedback-driven bringup.");
      }

      // ----------------------------------------------------------------------
      // Automatic power-on sequence.
      // Only send commands after the battery lifecycle node is ACTIVE, because
      // /hal/batterycontrol is created during battery configure.
      // ----------------------------------------------------------------------
      if (auto_power_on_ && initial_required_ready && !auto_12v_command_sent) {
        if (!auto_72v_command_sent) {
          if (send_battery_control_command(5, "72V ON")) {
            auto_72v_command_sent = true;
            auto_72v_command_time = Clock::now();
            RCLCPP_INFO(
              get_logger(),
              "72V ON command accepted. Waiting %ld ms before enabling 12V.",
              static_cast<long>(auto_12v_delay_.count()));
          }
        } else if (auto_72v_command_time.has_value() &&
          Clock::now() - *auto_72v_command_time >= auto_12v_delay_)
        {
          if (send_battery_control_command(1, "12V ON")) {
            auto_12v_command_sent = true;
            RCLCPP_INFO(
              get_logger(),
              "Automatic power-on sequence completed: 72V ON -> delay %ld ms -> 12V ON.",
              static_cast<long>(auto_12v_delay_.count()));
          }
        }
      }

      if (!power_groups_enabled) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      bool v48_on = false;
      bool v12_on = false;
      bool v24_on = false;
      bool v72_on = false;

      if (!require_power_feedback_) {
        // Test/bypass mode: treat every rail as available.
        v48_on = v12_on = v24_on = v72_on = true;
      } else {
        const auto msg = latest_battery_msg();
        if (msg.has_value()) {
          v48_on = msg->battery_voltage_48v * 0.1 >= min_48v_voltage_;
          v12_on = msg->switch_state_12v != 0;
          v24_on = msg->switch_state_24v != 0;
          v72_on = msg->switch_state_72v != 0;
        }
      }

      process_group("48V", rail_48v_nodes_, v48_on, rail_48v_runtime, power_stabilize_delay_);
      process_group("12V", rail_12v_nodes_, v12_on, rail_12v_runtime, power_stabilize_delay_);
      process_group("24V", rail_24v_nodes_, v24_on, rail_24v_runtime, power_stabilize_delay_);

      // Thruster must not be brought up unless BOTH rails are physically present.
      process_group(
        "thruster(72V+12V)", thruster_nodes_, v72_on && v12_on,
        thruster_runtime, power_stabilize_delay_);

      std::this_thread::sleep_for(100ms);
    }
  }

  void process_group(
    const std::string & group_name,
    const std::vector<std::string> & nodes,
    bool power_condition,
    GroupRuntime & runtime,
    std::chrono::milliseconds power_delay)
  {
    if (runtime.completed || nodes.empty()) {
      runtime.completed = true;
      return;
    }

    const auto now = Clock::now();

    // Power disappeared before activation: restart the power-stable timer.
    // We intentionally do not deactivate an already completed group here.
    if (!power_condition) {
      if (runtime.power_on_since.has_value() || has_pending_activation(runtime)) {
        RCLCPP_WARN(
          get_logger(), "%s power condition is not ready; pending bringup timer reset.",
          group_name.c_str());
      }
      runtime.power_on_since.reset();
      reset_pending_nodes(runtime);
      return;
    }

    if (!runtime.power_on_since.has_value()) {
      runtime.power_on_since = now;
      RCLCPP_INFO(
        get_logger(), "%s power condition detected; waiting %ld ms before configure.",
        group_name.c_str(), static_cast<long>(power_delay.count()));
      return;
    }

    if (now - *runtime.power_on_since < power_delay) {
      return;
    }

    bool all_active = true;
    for (const auto & node_name : nodes) {
      const auto node = normalize_node_name(node_name);
      auto & node_runtime = runtime.nodes[node];

      if (node_runtime.completed) {
        continue;
      }

      all_active = false;

      if (now < node_runtime.next_retry_at) {
        continue;
      }

      if (!node_runtime.activate_not_before.has_value()) {
        const auto result = ensure_configured(node);
        if (!result.has_value()) {
          node_runtime.next_retry_at = Clock::now() + lifecycle_bringup_retry_delay_;
          continue;
        }

        if (*result) {
          node_runtime.completed = true;
          RCLCPP_INFO(
            get_logger(), "%s lifecycle node is active: %s",
            group_name.c_str(), node.c_str());
          continue;
        }

        node_runtime.activate_not_before = Clock::now() + configure_to_activate_delay_;
        RCLCPP_INFO(
          get_logger(),
          "%s lifecycle node configured; waiting %ld ms before activate: %s",
          group_name.c_str(), static_cast<long>(configure_to_activate_delay_.count()),
          node.c_str());
        continue;
      }

      if (now < *node_runtime.activate_not_before) {
        continue;
      }

      const auto result = ensure_activated(node);
      if (!result.has_value()) {
        node_runtime.next_retry_at = Clock::now() + lifecycle_bringup_retry_delay_;
        continue;
      }

      if (*result) {
        node_runtime.completed = true;
        RCLCPP_INFO(
          get_logger(), "%s lifecycle node is active: %s",
          group_name.c_str(), node.c_str());
      } else {
        node_runtime.activate_not_before.reset();
        node_runtime.next_retry_at = Clock::now() + lifecycle_bringup_retry_delay_;
      }
    }

    all_active = true;
    for (const auto & node_name : nodes) {
      const auto node = normalize_node_name(node_name);
      const auto runtime_iter = runtime.nodes.find(node);
      if (runtime_iter == runtime.nodes.end() || !runtime_iter->second.completed) {
        all_active = false;
        break;
      }
    }

    if (all_active) {
      runtime.completed = true;
      RCLCPP_INFO(get_logger(), "%s lifecycle group is active.", group_name.c_str());
    }
  }

  static bool has_pending_activation(const GroupRuntime & runtime)
  {
    for (const auto & item : runtime.nodes) {
      if (!item.second.completed && item.second.activate_not_before.has_value()) {
        return true;
      }
    }
    return false;
  }

  static void reset_pending_nodes(GroupRuntime & runtime)
  {
    for (auto & item : runtime.nodes) {
      if (!item.second.completed) {
        item.second.activate_not_before.reset();
        item.second.next_retry_at = Clock::time_point::min();
      }
    }
  }

  bool required_initial_nodes_active()
  {
    for (const auto & node_name : initial_required_nodes_) {
      const auto node = normalize_node_name(node_name);
      const auto state = get_lifecycle_state(node);
      if (!state.has_value() ||
        *state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        return false;
      }
    }
    return true;
  }

  // Return value:
  //   true  -> node is already ACTIVE
  //   false -> node is configured/INACTIVE and still needs activation
  //   nullopt -> operation failed and should be retried later
  std::optional<bool> ensure_configured(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(lifecycle_transition_mutex_);
    const auto node = normalize_node_name(node_name);

    for (int attempt = 1; rclcpp::ok() && attempt <= lifecycle_bringup_max_attempts_; ++attempt) {
      const auto state = get_lifecycle_state(node);
      if (!state.has_value()) {
        RCLCPP_WARN(
          get_logger(), "Cannot read lifecycle state before configure (%d/%d): %s",
          attempt, lifecycle_bringup_max_attempts_, node.c_str());
      } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_INFO(get_logger(), "%s already ACTIVE; configure skipped.", node.c_str());
        return true;
      } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        RCLCPP_INFO(get_logger(), "%s already INACTIVE; configure skipped.", node.c_str());
        return false;
      } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        // Recheck immediately before sending configure to tolerate manual test actions.
        const auto before_configure = get_lifecycle_state(node);
        if (!before_configure.has_value()) {
          RCLCPP_WARN(get_logger(), "State recheck before configure failed: %s", node.c_str());
        } else if (*before_configure == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          RCLCPP_INFO(get_logger(), "%s became ACTIVE manually; configure skipped.", node.c_str());
          return true;
        } else if (*before_configure == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_INFO(get_logger(), "%s became INACTIVE manually; configure skipped.", node.c_str());
          return false;
        } else if (*before_configure == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
          RCLCPP_INFO(get_logger(), "Configuring lifecycle node: %s", node.c_str());
          if (change_lifecycle_state(
              node, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
          {
            return false;
          }
        } else {
          RCLCPP_WARN(
            get_logger(), "%s is in unsupported state %u before configure.",
            node.c_str(), static_cast<unsigned>(*before_configure));
          return std::nullopt;
        }
      } else {
        RCLCPP_WARN(
          get_logger(), "%s is in unsupported state %u during configure phase.",
          node.c_str(), static_cast<unsigned>(*state));
        return std::nullopt;
      }

      if (attempt < lifecycle_bringup_max_attempts_) {
        std::this_thread::sleep_for(lifecycle_bringup_retry_delay_);
      }
    }

    return std::nullopt;
  }

  // Return value:
  //   true  -> node is ACTIVE after this call
  //   false -> node is not activatable yet (e.g. returned to UNCONFIGURED)
  //   nullopt -> service/transition failure, retry later
  std::optional<bool> ensure_activated(const std::string & node_name)
  {
    std::lock_guard<std::mutex> lock(lifecycle_transition_mutex_);
    const auto node = normalize_node_name(node_name);

    for (int attempt = 1; rclcpp::ok() && attempt <= lifecycle_bringup_max_attempts_; ++attempt) {
      const auto state = get_lifecycle_state(node);
      if (!state.has_value()) {
        RCLCPP_WARN(
          get_logger(), "Cannot read lifecycle state before activate (%d/%d): %s",
          attempt, lifecycle_bringup_max_attempts_, node.c_str());
      } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_INFO(get_logger(), "%s already ACTIVE; activate skipped.", node.c_str());
        return true;
      } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        RCLCPP_WARN(
          get_logger(), "%s returned to UNCONFIGURED before activate; reconfigure required.",
          node.c_str());
        return false;
      } else if (*state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        // Recheck immediately before sending activate to tolerate manual test actions.
        const auto before_activate = get_lifecycle_state(node);
        if (!before_activate.has_value()) {
          RCLCPP_WARN(get_logger(), "State recheck before activate failed: %s", node.c_str());
        } else if (*before_activate == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
          RCLCPP_INFO(get_logger(), "%s became ACTIVE manually; activate skipped.", node.c_str());
          return true;
        } else if (*before_activate == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
          RCLCPP_WARN(
            get_logger(), "%s became UNCONFIGURED before activate; reconfigure required.",
            node.c_str());
          return false;
        } else if (*before_activate == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_INFO(get_logger(), "Activating lifecycle node: %s", node.c_str());
          if (change_lifecycle_state(
              node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
          {
            return true;
          }
        } else {
          RCLCPP_WARN(
            get_logger(), "%s is in unsupported state %u before activate.",
            node.c_str(), static_cast<unsigned>(*before_activate));
          return std::nullopt;
        }
      } else {
        RCLCPP_WARN(
          get_logger(), "%s is in unsupported state %u during activate phase.",
          node.c_str(), static_cast<unsigned>(*state));
        return std::nullopt;
      }

      if (attempt < lifecycle_bringup_max_attempts_) {
        std::this_thread::sleep_for(lifecycle_bringup_retry_delay_);
      }
    }

    return std::nullopt;
  }

  std::optional<uint8_t> get_lifecycle_state(const std::string & node)
  {
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client;
    {
      std::lock_guard<std::mutex> lock(lifecycle_client_mutex_);
      client = get_state_clients_[node];
      if (!client) {
        client = create_client<lifecycle_msgs::srv::GetState>(node + "/get_state");
        get_state_clients_[node] = client;
      }
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
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client;
    {
      std::lock_guard<std::mutex> lock(lifecycle_client_mutex_);
      client = change_state_clients_[node];
      if (!client) {
        client = create_client<lifecycle_msgs::srv::ChangeState>(node + "/change_state");
        change_state_clients_[node] = client;
      }
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

  bool autostart_{true};
  bool require_power_feedback_{true};
  bool auto_power_on_{true};
  bool worker_started_{false};
  bool shutting_down_{false};

  double min_48v_voltage_{36.0};
  std::chrono::milliseconds service_timeout_{5000};
  std::chrono::milliseconds auto_12v_delay_{1000};
  std::chrono::milliseconds power_stabilize_delay_{2000};
  std::chrono::milliseconds configure_to_activate_delay_{5000};
  int lifecycle_bringup_max_attempts_{3};
  std::chrono::milliseconds lifecycle_bringup_retry_delay_{1000};

  std::vector<std::string> initial_nodes_;
  std::vector<std::string> initial_required_nodes_;
  std::vector<std::string> rail_48v_nodes_;
  std::vector<std::string> rail_12v_nodes_;
  std::vector<std::string> rail_24v_nodes_;
  std::vector<std::string> thruster_nodes_;

  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::Subscription<hal::msg::HalBattery>::SharedPtr battery_sub_;
  rclcpp::Client<hal::srv::HalBatteryControlSrv>::SharedPtr battery_control_client_;
  rclcpp::Subscription<hal::msg::HalLifecyclestatesControl>::SharedPtr
    lifecycle_control_sub_;

  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>
    get_state_clients_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr>
    change_state_clients_;

  mutable std::mutex battery_mutex_;
  std::optional<hal::msg::HalBattery> latest_battery_;

  std::mutex lifecycle_client_mutex_;
  std::mutex lifecycle_transition_mutex_;

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