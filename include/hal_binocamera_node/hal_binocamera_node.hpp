#ifndef HAL_BINOCAMERA_NODE__HAL_BINOCAMERA_NODE_HPP_
#define HAL_BINOCAMERA_NODE__HAL_BINOCAMERA_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <hal/msg/halbinocamera_msg.hpp>
#include <hal/srv/halbinocamera_srv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hal_binocamera
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HalBinocameraNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  HalBinocameraNode();
  ~HalBinocameraNode() override;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  void declareParameters();
  bool openCamera();
  void closeCamera();
  void captureAndPublish();
  void publishStatus(std::uint8_t status_code);
  void handleToggleCamera(
    const std::shared_ptr<hal::srv::HalbinocameraSrv::Request> request,
    std::shared_ptr<hal::srv::HalbinocameraSrv::Response> response);
  void publishColorImage(
    const std::vector<std::uint8_t> & image_data,
    int width,
    int height,
    int stride);
  void publishDepthImage(
    const std::vector<std::uint16_t> & depth_data,
    int width,
    int height,
    int stride);

  struct Impl;
  std::unique_ptr<Impl> pimpl_;

  int grab_fail_count_;
  bool is_camera_open_;
  bool camera_enabled_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<hal::srv::HalbinocameraSrv>::SharedPtr toggle_camera_service_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp_lifecycle::LifecyclePublisher<hal::msg::HalbinocameraMsg>::SharedPtr status_pub_;
};

}  // namespace hal_binocamera

#endif  // HAL_BINOCAMERA_NODE__HAL_BINOCAMERA_NODE_HPP_