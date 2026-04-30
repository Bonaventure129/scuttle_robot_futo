#ifndef KEYBOARD_CONTROL_HPP
#define KEYBOARD_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <map>
#include <chrono>

class KeyboardControl : public rclcpp::Node
{
public:
    KeyboardControl();
    ~KeyboardControl();

private:
    void loop_callback();
    void set_terminal_raw();
    void restore_terminal();
    char read_key();

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Settings
    struct termios original_terminal_settings_;
    double linear_speed_ = 0.4;
    double angular_speed_ = 1.0;
    double persistence_ = 0.25; // Seconds to "remember" a key press

    // Timestamps for last key press
    rclcpp::Time last_up_time_;
    rclcpp::Time last_down_time_;
    rclcpp::Time last_left_time_;
    rclcpp::Time last_right_time_;
};

#endif // KEYBOARD_CONTROL_HPP