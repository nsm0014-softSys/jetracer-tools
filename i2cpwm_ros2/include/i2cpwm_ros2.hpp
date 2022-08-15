#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
extern "C" {
	#include <i2c/smbus.h>
}

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/logging.h"

// Data
#include <string>

// Message and Service Types
// Messages used for any service with no parameters
#include <std_srvs/srv/empty.hpp>
// messages used for drive movement topic
#include <geometry_msgs/msg/twist.hpp>

// messages used for the absolute and proportional movement topics
#include "i2cpwm_ros2_msgs/msg/servo.hpp"
#include "i2cpwm_ros2_msgs/msg/servo_array.hpp"
// messages used for the servo setup service
#include "i2cpwm_ros2_msgs/msg/servo_config.hpp"
#include "i2cpwm_ros2_msgs/msg/servo_config_array.hpp"
// request/response of the servo setup service
#include "i2cpwm_ros2_msgs/srv/servos_config.hpp"
// request/response of the drive mode service
#include "i2cpwm_ros2_msgs/srv/drive_mode.hpp"
#include "i2cpwm_ros2_msgs/msg/position.hpp"
#include "i2cpwm_ros2_msgs/msg/position_array.hpp"
// request/response of the integer parameter services
#include "i2cpwm_ros2_msgs/srv/int_value.hpp"



// Header only variables and structs
typedef struct _servo_config {
    int center;
    int range;
    int direction;
    int mode_pos;
} servo_config;

typedef struct _drive_mode {
	int mode;
	float rpm;
	float radius;
	float track;
	float scale;
} drive_mode;

enum drive_modes {
    MODE_UNDEFINED      = 0,
    MODE_ACKERMAN       = 1,
    MODE_DIFFERENTIAL   = 2,
    MODE_MECANUM        = 3,
    MODE_INVALID        = 4
};

enum drive_mode_positions {
    POSITION_UNDEFINED  = 0,
    POSITION_LEFTFRONT  = 1,
    POSITION_RIGHTFRONT = 2,
    POSITION_LEFTREAR   = 3,
    POSITION_RIGHTREAR  = 4,
    POSITION_INVALID    = 5
};

#define _BASE_ADDR   0x40
#ifndef _PI
#define _PI 3.14159265358979323846
#endif
#define _CONST(s) ((char*)(s))

enum pwm_regs {
  // Registers/etc.
  __MODE1              = 0x00,
  __MODE2              = 0x01,
  __SUBADR1            = 0x02,      // enable sub address 1 support
  __SUBADR2            = 0x03,      // enable sub address 2 support
  __SUBADR3            = 0x04,      // enable sub address 2 support
  __PRESCALE           = 0xFE,
  __CHANNEL_ON_L       = 0x06,
  __CHANNEL_ON_H       = 0x07,
  __CHANNEL_OFF_L      = 0x08,
  __CHANNEL_OFF_H      = 0x09,
  __ALL_CHANNELS_ON_L  = 0xFA,
  __ALL_CHANNELS_ON_H  = 0xFB,
  __ALL_CHANNELS_OFF_L = 0xFC,
  __ALL_CHANNELS_OFF_H = 0xFD,
  __RESTART            = 0x80,
  __SLEEP              = 0x10,      // enable low power mode
  __ALLCALL            = 0x01,
  __INVRT              = 0x10,      // invert the output control logic
  __OUTDRV             = 0x04
};

#define MAX_BOARDS 62
#define MAX_SERVOS (16*MAX_BOARDS)

// I2C CLASS
class i2cpwm_ros2 : public rclcpp::Node
{
    public:
        i2cpwm_ros2() : Node("i2cpwm_ros2")
        {
            static const rclcpp::Logger i2cpwm_ros2_LOGGER_ = rclcpp::get_logger("i2cpwm_ros2");   

            RCLCPP_INFO(rclcpp::get_logger("i2cpwm_ros2"), "Constructor: i2cpwm_ros2 pre-init");

            // Set Subscribers
            this->absolute_sub_ = this->create_subscription<i2cpwm_ros2_msgs::msg::ServoArray>
            ("/i2cpwm_ros2/servos_absolute", 10, std::bind(&i2cpwm_ros2::servos_absolute, this, std::placeholders::_1));
            this->relative_sub_ = this->create_subscription<i2cpwm_ros2_msgs::msg::ServoArray>
            ("/i2cpwm_ros2/servos_proportional", 10, std::bind(&i2cpwm_ros2::servos_proportional, this, std::placeholders::_1));
            this->drive_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
            ("/i2cpwm_ros2/servos_drive", 10, std::bind(&i2cpwm_ros2::servos_drive, this, std::placeholders::_1));

            // Set up Services
            this->frequency_srv_ = this->create_service<i2cpwm_ros2_msgs::srv::IntValue>
            ("i2cpwm_ros2/set_pwm_frequency", std::bind(&i2cpwm_ros2::set_pwm_frequency, this, std::placeholders::_1, std::placeholders::_2));
            this->config_srv_ = this->create_service<i2cpwm_ros2_msgs::srv::ServosConfig>
            ("i2cpwm_ros2/config_servos", std::bind(&i2cpwm_ros2::config_servos, this, std::placeholders::_1, std::placeholders::_2));
            this->mode_srv_ = this->create_service<i2cpwm_ros2_msgs::srv::DriveMode>
            ("i2cpwm_ros2/config_drive_mode", std::bind(&i2cpwm_ros2::config_drive_mode, this, std::placeholders::_1, std::placeholders::_2));
            this->stop_srv_ = this->create_service<std_srvs::srv::Empty>
            ("i2cpwm_ros2/stop_servos", std::bind(&i2cpwm_ros2::stop_servos, this, std::placeholders::_1, std::placeholders::_2));
            
            // Add parameters to parameter service
            this->declare_parameter("logger_level", "DEBUG");

            this->declare_parameter("i2c_device_address", "/dev/i2c-");
            this->declare_parameter("controller_io_device", 0);
            this->declare_parameter("controller_io_handle", 0);
            this->declare_parameter("pwm_frequency", 50);
            this->declare_parameter("active_board", 1);
                // Drive Mode Params
                this->declare_parameter("drive_config.mode", 3); // Mecanum drive mode
                this->declare_parameter("drive_config.radius", 0.062);
                this->declare_parameter("drive_config.rpm", 60.0);
                this->declare_parameter("drive_config.scale", 0.3);
                this->declare_parameter("drive_config.track", 0.2);
                    // Servo configs
                    this->declare_parameter("drive_config.servos.servo1.number", 1);
                    this->declare_parameter("drive_config.servos.servo1.position", 1);


            // get values from parameter service and set local variables
            this->_controller_io_device = this->get_parameter("controller_io_device").as_int();
            this->_controller_io_handle = this->get_parameter("controller_io_handle").as_int();
            this->_pwm_frequency = this->get_parameter("pwm_frequency").as_int();
            this->_active_board = this->get_parameter("active_board").as_int();

            // Do some setup on start
            this->_i2cpwm_ros2_setup();	
            RCLCPP_INFO(rclcpp::get_logger("i2cpwm_ros2"), "Constructor: i2cpwm_ros2 init");
        }
        
        // subscriber functions
        void servos_absolute (const i2cpwm_ros2_msgs::msg::ServoArray::SharedPtr msg);
        void servos_proportional(const i2cpwm_ros2_msgs::msg::ServoArray::SharedPtr msg);
        void servos_drive (const geometry_msgs::msg::Twist::SharedPtr msg);

        // service functions
        void set_pwm_frequency (const std::shared_ptr<i2cpwm_ros2_msgs::srv::IntValue::Request> req,
            std::shared_ptr<i2cpwm_ros2_msgs::srv::IntValue::Response> res);
        void config_servos (const std::shared_ptr<i2cpwm_ros2_msgs::srv::ServosConfig::Request> req,
            std::shared_ptr<i2cpwm_ros2_msgs::srv::ServosConfig::Response> res);
        void config_drive_mode (const std::shared_ptr<i2cpwm_ros2_msgs::srv::DriveMode::Request> req,
            std::shared_ptr<i2cpwm_ros2_msgs::srv::DriveMode::Response> res);
        void stop_servos (const std::shared_ptr<std_srvs::srv::Empty::Request> req,
            std::shared_ptr<std_srvs::srv::Empty::Response> res);

        drive_mode _active_drive;					// used when converting Twist geometry to PWM values and which servos are for motion
        int _last_servo = -1;
        int _pwm_boards[MAX_BOARDS];                // we can support up to 62 boards (1..62)
        int _active_board = 0;                      // used to determine if I2C SLAVE change is needed
        int _controller_io_handle;                  // linux file handle for I2C
        int _controller_io_device;                  // linux file for I2C
        int _pwm_frequency; 
        servo_config _servo_configs[MAX_SERVOS];
        std::string _device;

        // Timers
        // none for now, maybe later

        // Subscribers
        rclcpp::Subscription<i2cpwm_ros2_msgs::msg::ServoArray>::SharedPtr absolute_sub_;
        rclcpp::Subscription<i2cpwm_ros2_msgs::msg::ServoArray>::SharedPtr relative_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_sub_;

        // Services
        rclcpp::Service<i2cpwm_ros2_msgs::srv::IntValue>::SharedPtr frequency_srv_;
        rclcpp::Service<i2cpwm_ros2_msgs::srv::ServosConfig>::SharedPtr config_srv_;
        rclcpp::Service<i2cpwm_ros2_msgs::srv::DriveMode>::SharedPtr mode_srv_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;

        // Logging tools
        std::string _logging_level_str;

        
        // Public constructor/destructor for later implementation
        ~i2cpwm_ros2()
        {
            RCLCPP_INFO(rclcpp::get_logger("i2cpwm_ros2"), "Destructor: i2cpwm_ros2");
        };

    private:
        float _abs(float v1);
        float _min(float v1, float v2);
        float _max(float v1, float v2);
        float _absmin(float v1, float v2);
        float _absmax(float v1, float v2);

        int _smoothing(float speed);
        float _convert_mps_to_proportional (float speed);
        void _set_pwm_frequency (int freq);
        void _set_pwm_interval_all (int start, int end);
        void _set_active_board (int board);
        void _set_pwm_interval (int servo, int start, int end);
        void _set_pwm_interval_proportional (int servo, float value);
        void _config_servo (int servo, int center, int range, int direction);
        int _config_servo_position (int servo, int position);
        int _config_drive_mode (std::string mode, float rpm, float radius, float track, float scale);
        // _init is a carryover function from previous code
        void _init (const char* filename);
        // _i2cpwm_ros2_setup is for setting up servos on startup, not the same as _init
        void _i2cpwm_ros2_setup();
        void handle_logger_level();
};














