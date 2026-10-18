#include "kushagrabot_hardware/system_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kushagrabot_hardware
{

hardware_interface::CallbackReturn KushagraBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_wheel_pos_ = 0.0;
  left_wheel_vel_ = 0.0;
  right_wheel_pos_ = 0.0;
  right_wheel_vel_ = 0.0;
  
  left_wheel_cmd_ = 0.0;
  right_wheel_cmd_ = 0.0;

  left_ticks_ = 0;
  right_ticks_ = 0;
  prev_left_ticks_ = 0;
  prev_right_ticks_ = 0;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KushagraBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KushagraBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KushagraBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize ROS 2 Node for micro-ROS communication
  node_ = std::make_shared<rclcpp::Node>("kushagrabot_hardware_interface_node");

  left_wheel_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/right_wheel_cmd", 10);
  right_wheel_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32>("/left_wheel_cmd", 10);

  left_ticks_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/right_ticks", 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) { left_ticks_ = msg->data; });

  right_ticks_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
    "/left_ticks", 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) { right_ticks_ = msg->data; });

  executor_.add_node(node_);
  executor_thread_ = std::make_shared<std::thread>([this]() { executor_.spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KushagraBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &left_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_vel_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_POSITION, &right_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_vel_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KushagraBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &left_wheel_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &right_wheel_cmd_));

  return command_interfaces;
}

hardware_interface::CallbackReturn KushagraBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  left_wheel_cmd_ = 0.0;
  right_wheel_cmd_ = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KushagraBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type KushagraBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();

  long d_left = left_ticks_ - prev_left_ticks_;
  long d_right = right_ticks_ - prev_right_ticks_;

  prev_left_ticks_ = left_ticks_;
  prev_right_ticks_ = right_ticks_;

  double d_left_rad = (d_left / ENCODER_CPR) * 2.0 * M_PI;
  double d_right_rad = (d_right / ENCODER_CPR) * 2.0 * M_PI;

  left_wheel_pos_ += d_left_rad;
  right_wheel_pos_ += d_right_rad;

  if (dt > 0.0) {
    left_wheel_vel_ = d_left_rad / dt;
    right_wheel_vel_ = d_right_rad / dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KushagraBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std_msgs::msg::Float32 left_msg;
  left_msg.data = left_wheel_cmd_;
  left_wheel_cmd_pub_->publish(left_msg);

  std_msgs::msg::Float32 right_msg;
  right_msg.data = right_wheel_cmd_;
  right_wheel_cmd_pub_->publish(right_msg);

  return hardware_interface::return_type::OK;
}

}  // namespace kushagrabot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kushagrabot_hardware::KushagraBotSystemHardware, hardware_interface::SystemInterface)
