
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class BspArmTrajectoryNode : public rclcpp::Node
{
public:
    explicit BspArmTrajectoryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("bsp_arm_trajectory_node", options)
    {
        // ============================================================
        // 参数说明
        //
        // joint_state_topic:
        //   订阅 HAL 发布的当前关节状态
        //
        // target_joint_topic:
        //   订阅上位机/BSP 下发的最终目标点
        //
        // joint_cmd_topic:
        //   发布给 HAL 的每周期插值目标点
        //
        // publish_rate_hz:
        //   插值点发布频率，建议与 HAL 控制周期一致或略低
        //
        // default_duration_sec:
        //   如果目标点没有设置 time_from_start，则使用默认运动时间
        // ============================================================
        this->declare_parameter<std::string>(
            "joint_state_topic",
            "/left_arm/hal/manipulator/joint_states");

        this->declare_parameter<std::string>(
            "target_joint_topic",
            "/left_arm/bsp/manipulator/target_joint");

        this->declare_parameter<std::string>(
            "joint_cmd_topic",
            "/left_arm/hal/manipulator/joint_cmd");

        this->declare_parameter<double>("publish_rate_hz", 50.0);
        this->declare_parameter<double>("default_duration_sec", 3.0);

        joint_state_topic_ =
            this->get_parameter("joint_state_topic").as_string();

        target_joint_topic_ =
            this->get_parameter("target_joint_topic").as_string();

        joint_cmd_topic_ =
            this->get_parameter("joint_cmd_topic").as_string();

        publish_rate_hz_ =
            this->get_parameter("publish_rate_hz").as_double();

        default_duration_sec_ =
            this->get_parameter("default_duration_sec").as_double();

        if (publish_rate_hz_ <= 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "publish_rate_hz <= 0, reset to 50.0");
            publish_rate_hz_ = 50.0;
        }

        if (default_duration_sec_ <= 0.0) {
            RCLCPP_WARN(
                get_logger(),
                "default_duration_sec <= 0, reset to 3.0");
            default_duration_sec_ = 3.0;
        }

        // ============================================================
        // 订阅 HAL 当前关节状态
        // ============================================================
        joint_state_sub_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                joint_state_topic_,
                rclcpp::QoS(10),
                std::bind(
                    &BspArmTrajectoryNode::joint_state_callback,
                    this,
                    std::placeholders::_1));

        // ============================================================
        // 订阅上位机/BSP 目标点
        // ============================================================
        target_joint_sub_ =
            this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
                target_joint_topic_,
                rclcpp::QoS(10),
                std::bind(
                    &BspArmTrajectoryNode::target_joint_callback,
                    this,
                    std::placeholders::_1));

        // ============================================================
        // 发布插值后的目标点给 HAL
        // ============================================================
        joint_cmd_pub_ =
            this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
                joint_cmd_topic_,
                rclcpp::QoS(10));

        const auto period =
            std::chrono::duration<double>(1.0 / publish_rate_hz_);

        timer_ =
            this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&BspArmTrajectoryNode::timer_callback, this));

        RCLCPP_INFO(get_logger(), "BSP arm trajectory node started.");
        RCLCPP_INFO(get_logger(), "Subscribe joint state: %s", joint_state_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Subscribe target joint: %s", target_joint_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Publish joint cmd: %s", joint_cmd_topic_.c_str());
    }

private:
    // ============================================================
    // 接收 HAL 当前关节状态
    // ============================================================
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.empty()) {
            return;
        }

        current_position_ = msg->position;
        has_current_position_ = true;
    }

    // ============================================================
    // 接收目标点
    //
    // 上位机只需要发最终目标点，例如：
    // positions = [1.0, 0.5, -0.3, 0.2, 0.1]
    // time_from_start = 3s
    //
    // BSP 收到后，从当前 joint_states 作为起点，
    // 用五次多项式生成中间点。
    // ============================================================
    void target_joint_callback(
        const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg)
    {
        if (msg->positions.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty target positions.");
            return;
        }

        if (!has_current_position_) {
            RCLCPP_WARN(
                get_logger(),
                "No current joint state received yet. Ignore target.");
            return;
        }

        if (msg->positions.size() != current_position_.size()) {
            RCLCPP_ERROR(
                get_logger(),
                "Target size (%zu) != current joint size (%zu).",
                msg->positions.size(),
                current_position_.size());
            return;
        }

        start_position_ = current_position_;
        target_position_ = msg->positions;

        double duration =
            static_cast<double>(msg->time_from_start.sec) +
            static_cast<double>(msg->time_from_start.nanosec) * 1e-9;

        if (duration <= 0.0) {
            duration = default_duration_sec_;
        }

        trajectory_duration_sec_ = duration;
        trajectory_start_time_ = this->now();
        trajectory_active_ = true;

        RCLCPP_INFO(
            get_logger(),
            "New trajectory accepted. joint_count=%zu, duration=%.3f sec",
            target_position_.size(),
            trajectory_duration_sec_);
    }

    // ============================================================
    // 五次多项式插值函数
    //
    // s = t / T
    // q(t) = q0 + (qT - q0) * (10s^3 - 15s^4 + 6s^5)
    //
    // 特点：
    // 起点速度为 0
    // 终点速度为 0
    // 起点加速度为 0
    // 终点加速度为 0
    // ============================================================
    double quintic_interpolate(double q0, double qT, double t, double T) const
    {
        if (T <= 0.0) {
            return qT;
        }

        double s = std::clamp(t / T, 0.0, 1.0);

        double s2 = s * s;
        double s3 = s2 * s;
        double s4 = s3 * s;
        double s5 = s4 * s;

        double blend = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;

        return q0 + (qT - q0) * blend;
    }

    // ============================================================
    // 定时发布插值点给 HAL
    // ============================================================
    void timer_callback()
    {
        if (!trajectory_active_) {
            return;
        }

        if (start_position_.size() != target_position_.size()) {
            RCLCPP_ERROR(get_logger(), "Trajectory size mismatch. Stop trajectory.");
            trajectory_active_ = false;
            return;
        }

        rclcpp::Time now = this->now();
        double t = (now - trajectory_start_time_).seconds();

        trajectory_msgs::msg::JointTrajectoryPoint cmd_msg;
        cmd_msg.positions.resize(target_position_.size(), 0.0);

        if (t >= trajectory_duration_sec_) {
            // 到达终点，发布最终目标点
            cmd_msg.positions = target_position_;
            joint_cmd_pub_->publish(cmd_msg);

            trajectory_active_ = false;

            RCLCPP_INFO(get_logger(), "Trajectory finished.");
            return;
        }

        for (size_t i = 0; i < target_position_.size(); ++i) {
            cmd_msg.positions[i] =
                quintic_interpolate(
                    start_position_[i],
                    target_position_[i],
                    t,
                    trajectory_duration_sec_);
        }

        joint_cmd_pub_->publish(cmd_msg);
    }

private:
    std::string joint_state_topic_;
    std::string target_joint_topic_;
    std::string joint_cmd_topic_;

    double publish_rate_hz_{50.0};
    double default_duration_sec_{3.0};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr target_joint_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr joint_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> current_position_;
    std::vector<double> start_position_;
    std::vector<double> target_position_;

    bool has_current_position_{false};
    bool trajectory_active_{false};

    rclcpp::Time trajectory_start_time_;
    double trajectory_duration_sec_{3.0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BspArmTrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}