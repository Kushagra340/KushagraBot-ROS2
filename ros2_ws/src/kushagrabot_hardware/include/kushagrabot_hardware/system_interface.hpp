#ifndef KUSHAGRABOT_HARDWARE__SYSTEM_INTERFACE_HPP_
#define KUSHAGRABOT_HARDWARE__SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

namespace kushagrabot_hardware
{

class KushagraBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KushagraBotSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ROS 2 Node for communicating with ESP32 via Micro-ROS
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<std::thread> executor_thread_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Publishers and Subscribers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wheel_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wheel_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_ticks_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_ticks_sub_;

  // Hardware states and commands
  double left_wheel_pos_;
  double left_wheel_vel_;
  double right_wheel_pos_;
  double right_wheel_vel_;

  double left_wheel_cmd_;
  double right_wheel_cmd_;

  // Encoder values
  long left_ticks_;
  long right_ticks_;
  long prev_left_ticks_;
  long prev_right_ticks_;

  // Physical constants
  const double ENCODER_CPR = 353.0;
};

}  // namespace kushagrabot_hardware

#endif  // KUSHAGRABOT_HARDWARE__SYSTEM_INTERFACE_HPP_
