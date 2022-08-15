// C++ STL
#include <chrono>
#include <thread>
#include <vector>
#include <cstdlib>
#include <memory>
using namespace std::chrono_literals;


// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

// Needed external messages 
#include "i2cpwm_ros2_msgs/msg/servo.hpp"
#include "i2cpwm_ros2_msgs/msg/servo_array.hpp"
#include "i2cpwm_ros2_msgs/srv/servos_config.hpp"

#include "softsys_msgs/msg/throttle.hpp"
#include "softsys_msgs/msg/steer.hpp"
#include "softsys_msgs/msg/safety.hpp"

// Joystick message type, from ROS2
#include <sensor_msgs/msg/joy.hpp>

class softsys_joy_node : public rclcpp::Node
{
    private:

        // C++ Threading
        std::vector<std::thread> softsys_threads_;

        // Subscribe to controller input from Joy Node
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_input_sub_;

        // Subscribe to external throttle and steer commands for autonomy
        rclcpp::Subscription<softsys_msgs::msg::Throttle>::SharedPtr throttle_input_sub_;
        rclcpp::Subscription<softsys_msgs::msg::Steer>::SharedPtr steer_input_sub_;

        // Publish to servos
        rclcpp::Publisher<i2cpwm_ros2_msgs::msg::ServoArray>::SharedPtr i2cpwm_pub_;
        rclcpp::Publisher<softsys_msgs::msg::Safety>::SharedPtr safety_pub_;

        // Create service client to set servo parameters
        rclcpp::Client<i2cpwm_ros2_msgs::srv::ServosConfig>::SharedPtr i2cpwm_srv_client_;


        // Timer to publish to servos
        rclcpp::TimerBase::SharedPtr joy_heartbeat_timer_;

        // Private Variables
        // // three bools to separate steering, throttle and overall control modes
        bool isAutoSteer_; 
        bool isAutoThrottle_;
        bool isRecievingAutoThrottle_;
        bool isRecievingAutoSteer_;


        bool isAuto_;
        bool canSwitchMode_;
        bool isJoyPublished_;

        bool eStop_; // true to apply breaks immediately if needed

        // timestamps for controller heart beat
        rclcpp::Time joy_msg_time_;
        rclcpp::Time current_time_;
        rclcpp::Time throttle_time_;
        rclcpp::Time steering_time_;

        double joy_timeout_; // parameter for timeout length before estop, in seconds
        std::chrono::milliseconds joy_timeout_ms_; // millisecond timeout period for timer to use
        int sec_to_ms_ = 1000;

        // timeout for autonomous driving inputs
        double auto_timeout_;
        std::chrono::milliseconds auto_timeout_ms_;

        // Messages to publish to servos
        i2cpwm_ros2_msgs::msg::ServoArray servo_array_msg_;
        i2cpwm_ros2_msgs::msg::Servo steer_servo_msg_;
        i2cpwm_ros2_msgs::msg::Servo throttle_servo_msg_;

        softsys_msgs::msg::Safety safety_msg_;

        // Servo data in terms of PWM parameters
        double autonomous_switching_delay_;
        int autonomous_reset_count_;
        int steer_servo_num_;
        int drive_servo_num_;
        double servo_update_rate_; // time in seconds to execute the timer
        std::chrono::milliseconds servo_update_rate_ms_;

        int max_steer_val_;
        int max_throttle_val_;
        int min_steer_val_;
        int min_throttle_val_;
        int steer_range_;
        int throttle_range_;

        // Call back functions for subscribers
        void controller_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void throttle_callback(const softsys_msgs::msg::Throttle::SharedPtr msg);
        void steer_callback(const softsys_msgs::msg::Steer::SharedPtr msg);

        // Timer callback
        void joy_timer_callback_();

        // Class functions
        void publish_servo_data();
        void reset_autonomous_count();
        void send_servo_config_req();
        void publish_safety_data();

    public:
        softsys_joy_node();
        ~softsys_joy_node();
};


