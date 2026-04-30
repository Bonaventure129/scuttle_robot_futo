#include "scuttle_controller/keyboard_control.hpp"
#include <fcntl.h>
#include <poll.h>
#include <signal.h>

// Global pointer for signal handler to restore terminal
struct termios* global_settings_ptr = nullptr;

void signal_handler(int signum) {
    if (global_settings_ptr != nullptr) {
        tcsetattr(STDIN_FILENO, TCSANOW, global_settings_ptr);
    }
    rclcpp::shutdown();
    exit(signum);
}

KeyboardControl::KeyboardControl() : Node("keyboard_control")
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/key_vel", 10);

    // Initialize timestamps properly
    rcl_clock_type_t clock_type = this->get_clock()->get_clock_type();
    last_up_time_ = rclcpp::Time(0, 0, clock_type);
    last_down_time_ = rclcpp::Time(0, 0, clock_type);
    last_left_time_ = rclcpp::Time(0, 0, clock_type);
    last_right_time_ = rclcpp::Time(0, 0, clock_type);

    set_terminal_raw();

    // Register Ctrl+C handler
    global_settings_ptr = &original_terminal_settings_;
    signal(SIGINT, signal_handler);

    RCLCPP_INFO(this->get_logger(), "--------------------------------");
    RCLCPP_INFO(this->get_logger(), " C++ DIAGONAL TELEOP (SAFE MODE)");
    RCLCPP_INFO(this->get_logger(), " Use ARROW KEYS. 'q' to quit.");
    RCLCPP_INFO(this->get_logger(), "--------------------------------");

    // Run loop at 20Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&KeyboardControl::loop_callback, this));
}

KeyboardControl::~KeyboardControl()
{
    restore_terminal();
}

void KeyboardControl::set_terminal_raw()
{
    tcgetattr(STDIN_FILENO, &original_terminal_settings_);
    struct termios raw = original_terminal_settings_;
    raw.c_lflag &= ~(ICANON | ECHO); // Disable line buffering and echo
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

void KeyboardControl::restore_terminal()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_settings_);
}

void KeyboardControl::loop_callback()
{
    // 1. USE POLL TO CHECK FOR INPUT SAFELY
    struct pollfd fds[1];
    fds[0].fd = STDIN_FILENO;
    fds[0].events = POLLIN; // Check for data to read

    char c;
    std::string key_buffer = "";
    
    // poll() returns > 0 if data is waiting. Timeout is 0ms (instant).
    if (poll(fds, 1, 0) > 0) {
        while (read(STDIN_FILENO, &c, 1) > 0) {
            key_buffer += c;
        }
    }

    // 2. Parse Keys
    auto now = this->now();
    bool key_pressed = false;

    // DEBUG: Uncomment this line if you suspect keys aren't detected
    // if (!key_buffer.empty()) RCLCPP_INFO(this->get_logger(), "Key bytes: %lu", key_buffer.length());

    if (key_buffer.find("\x1b[A") != std::string::npos || key_buffer.find("\x1bOA") != std::string::npos) {
        last_up_time_ = now;
        key_pressed = true;
    }
    if (key_buffer.find("\x1b[B") != std::string::npos || key_buffer.find("\x1bOB") != std::string::npos) {
        last_down_time_ = now;
        key_pressed = true;
    }
    if (key_buffer.find("\x1b[C") != std::string::npos || key_buffer.find("\x1bOC") != std::string::npos) {
        last_right_time_ = now;
        key_pressed = true;
    }
    if (key_buffer.find("\x1b[D") != std::string::npos || key_buffer.find("\x1bOD") != std::string::npos) {
        last_left_time_ = now;
        key_pressed = true;
    }
    if (key_buffer.find('q') != std::string::npos) {
        rclcpp::shutdown();
        return;
    }

    // 3. Logic
    geometry_msgs::msg::Twist twist;
    try {
        double seconds_since_up = (now - last_up_time_).seconds();
        double seconds_since_down = (now - last_down_time_).seconds();
        double seconds_since_left = (now - last_left_time_).seconds();
        double seconds_since_right = (now - last_right_time_).seconds();

        if (seconds_since_up < persistence_) twist.linear.x = linear_speed_;
        else if (seconds_since_down < persistence_) twist.linear.x = -linear_speed_;

        if (seconds_since_left < persistence_) twist.angular.z = angular_speed_;
        else if (seconds_since_right < persistence_) twist.angular.z = -angular_speed_;
        
    } catch (...) {
        // Suppress time errors
    }

    publisher_->publish(twist);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}