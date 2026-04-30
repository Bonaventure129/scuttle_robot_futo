#pragma once

#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace scuttle_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ScuttleInterface : public hardware_interface::SystemInterface
{
public:
  ScuttleInterface();
  virtual ~ScuttleInterface();

  // LifecycleNodeInterface overrides
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // SystemInterface overrides
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  LibSerial::SerialPort arduino_;
  std::string port_;
  
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  // Store the correct indices for the wheels found in URDF
  size_t right_wheel_index_ = 0;
  size_t left_wheel_index_ = 0;
  
  rclcpp::Time last_run_;
};

}  // namespace scuttle_firmware