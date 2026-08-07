#include "scuttle_firmware/scuttle_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace scuttle_firmware;
using namespace hardware_interface;
using namespace rclcpp;
using namespace rclcpp_lifecycle;
using namespace rclcpp_lifecycle::node_interfaces;

ScuttleInterface::ScuttleInterface()
{
}

ScuttleInterface::~ScuttleInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ScuttleInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}

CallbackReturn ScuttleInterface::on_init(const HardwareInfo &hardware_info)
{
  // Call the base class implementation.
  // Note: This might generate a deprecation warning in Jazzy logs, 
  // but it is the correct and compiling way to initialize.
  if (SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ScuttleInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  // Resize vectors for ALL joints (wheels + casters)
  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);

  // --- FIND CORRECT INDICES BY NAME ---
  bool found_right = false;
  bool found_left = false;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].name == "r_wheel_joint")
    {
      right_wheel_index_ = i;
      found_right = true;
    }
    else if (info_.joints[i].name == "l_wheel_joint")
    {
      left_wheel_index_ = i;
      found_left = true;
    }
  }

  if (!found_right || !found_left)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ScuttleInterface"), 
                 "Could not find joints 'r_wheel_joint' or 'l_wheel_joint' in URDF");
    return CallbackReturn::FAILURE;
  }
  
  last_run_ = Clock().now();

  return CallbackReturn::SUCCESS;
}

vector<StateInterface> ScuttleInterface::export_state_interfaces()
{
  vector<StateInterface> state_interfaces;

  // Export state interfaces for ALL joints
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

vector<CommandInterface> ScuttleInterface::export_command_interfaces()
{
  vector<CommandInterface> command_interfaces;

  // Only export command interfaces for the DRIVEN WHEELS using stored indices
  command_interfaces.emplace_back(CommandInterface(
      info_.joints[right_wheel_index_].name, HW_IF_VELOCITY, &velocity_commands_[right_wheel_index_]));
      
  command_interfaces.emplace_back(CommandInterface(
      info_.joints[left_wheel_index_].name, HW_IF_VELOCITY, &velocity_commands_[left_wheel_index_]));

  return command_interfaces;
}

CallbackReturn ScuttleInterface::on_activate(const State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ScuttleInterface"), "Starting robot hardware ...");

  fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  fill(position_states_.begin(), position_states_.end(), 0.0);
  fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200); // Changed to match SoftwareSerial
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ScuttleInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ScuttleInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ScuttleInterface::on_deactivate(const State &)
{
  RCLCPP_INFO(rclcpp::get_logger("ScuttleInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ScuttleInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ScuttleInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ScuttleInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  string message;
  
  try {
    // Drop the timeout to 20ms! If it waits longer than this, 
    // it will break the 50ms (20Hz) controller_manager deadline.
    arduino_.ReadLine(message, '\n', 20);
  } catch (const LibSerial::ReadTimeout&) {
    // The Pi and Arduino clocks drifted out of phase this cycle.
    // Do NOT zero out the velocity here! It will cause jerky odometry 
    // and make Nav2 think the robot is constantly crashing.
    // Just return and let the virtual robot coast on the previous velocity.
    return hardware_interface::return_type::OK;
  }

  auto dt = (Clock().now() - last_run_).seconds();
  stringstream ss(message);
  string res;
  int multiplier = 1;
  
  while(getline(ss, res, ','))
  {
    if (res.empty()) continue;
    if (res.length() < 2) continue;

    multiplier = (res.at(1) == 'p') ? 1 : -1;

    if(res.at(0) == 'r')
    {
      velocity_states_.at(right_wheel_index_) = multiplier * std::stod(res.substr(2));
      position_states_.at(right_wheel_index_) += velocity_states_.at(right_wheel_index_) * dt;
    }
    else if(res.at(0) == 'l')
    {
      velocity_states_.at(left_wheel_index_) = multiplier * std::stod(res.substr(2));
      position_states_.at(left_wheel_index_) += velocity_states_.at(left_wheel_index_) * dt;
    }
  }
  
  // Notice that we only update the last_run_ clock if we successfully read data.
  // This ensures the math integrates perfectly even if we skip a cycle!
  last_run_ = Clock().now();
  
  return hardware_interface::return_type::OK;
}

return_type ScuttleInterface::write(const Time &, const Duration &)
{
  stringstream message_stream;
  
  // Use specific indices
  double right_vel = velocity_commands_.at(right_wheel_index_);
  double left_vel = velocity_commands_.at(left_wheel_index_);

  char right_wheel_sign = right_vel >= 0 ? 'p' : 'n';
  char left_wheel_sign = left_vel >= 0 ? 'p' : 'n';
  
  string compensate_zeros_right = (abs(right_vel) < 10.0) ? "0" : "";
  string compensate_zeros_left = (abs(left_vel) < 10.0) ? "0" : "";
  
  message_stream << fixed << setprecision(2) << 
    "r" << right_wheel_sign << compensate_zeros_right << abs(right_vel) << 
    ",l" <<  left_wheel_sign << compensate_zeros_left << abs(left_vel) << ",\n";

  try
  {
    arduino_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ScuttleInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port " << port_);
    return return_type::ERROR;
  }

  return return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(scuttle_firmware::ScuttleInterface, SystemInterface)