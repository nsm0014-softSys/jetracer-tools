#include <softsys_joy_node.hpp>

softsys_joy_node::softsys_joy_node() : Node("softsys_joy_node")
{
  // Setup logger
  static const rclcpp::Logger softsys_joy_LOGGER_ = rclcpp::get_logger("softsys_joy_node");

  // Set Subscribers
  this->controller_input_sub_ = this->create_subscription<sensor_msgs::msg::Joy>
      ("joy", 10, std::bind(&softsys_joy_node::controller_callback, this, std::placeholders::_1));
  this->throttle_input_sub_ = this->create_subscription<softsys_msgs::msg::Throttle>
      ("softsys/throttle_cmd", 10, std::bind(&softsys_joy_node::throttle_callback, this, std::placeholders::_1));
  this->steer_input_sub_ = this->create_subscription<softsys_msgs::msg::Steer>
      ("softsys/steer_cmd", 10, std::bind(&softsys_joy_node::steer_callback, this, std::placeholders::_1));

  // Set servo publisher
  this->i2cpwm_pub_ = this->create_publisher<i2cpwm_ros2_msgs::msg::ServoArray>("/i2cpwm_ros2/servos_absolute", 20);
  this->safety_pub_ = this->create_publisher<softsys_msgs::msg::Safety>("/softsys/safety", 20);

  // Set servo service client
  this->i2cpwm_srv_client_ = this->create_client<i2cpwm_ros2_msgs::srv::ServosConfig>("i2cpwm_ros2/config_servos");

  // Declare and set params
  this->declare_parameter("steer_servo_num", 2);
  this->steer_servo_num_ = this->get_parameter("steer_servo_num").as_int();

  this->declare_parameter("drive_servo_num", 1);
  this->drive_servo_num_ = this->get_parameter("drive_servo_num").as_int();

  this->declare_parameter("joy_timeout", 1.0);
  this->joy_timeout_ = this->get_parameter("joy_timeout").as_double();

  this->declare_parameter("auto_timeout", 1.0);
  this->auto_timeout_ = this->get_parameter("auto_timeout").as_double();

  this->declare_parameter("min_steer_value", 305);
  this->min_steer_val_ = this->get_parameter("min_steer_value").as_int();

  this->declare_parameter("max_steer_value", 330);
  this->max_steer_val_ = this->get_parameter("max_steer_value").as_int();

  this->declare_parameter("min_throttle_value", 305);
  this->min_throttle_val_ = this->get_parameter("min_throttle_value").as_int();

  this->declare_parameter("max_throttle_value", 330);
  this->max_throttle_val_ = this->get_parameter("max_throttle_value").as_int();

  this->declare_parameter("servo_update_rate_seconds", 0.01);
  this->servo_update_rate_ = this->get_parameter("servo_update_rate_seconds").as_double();

  this->declare_parameter("autonomous_switching_delay", 1.0);
  this->autonomous_switching_delay_ = this->get_parameter("autonomous_switching_delay").as_double();

  // Set timer
  this->servo_update_rate_ms_ = std::chrono::milliseconds((int)(this->sec_to_ms_*this->servo_update_rate_));
  this->joy_heartbeat_timer_ = this->create_wall_timer(this->servo_update_rate_ms_, std::bind(&softsys_joy_node::joy_timer_callback_, this));

  // set variables
  this->isAutoSteer_ = false;
  this->isAutoThrottle_ = false;
  this->isRecievingAutoThrottle_ = false;
  this->isRecievingAutoSteer_ = false;
  this->isAuto_ = false;

  this->canSwitchMode_ = true;
  this->eStop_ = false; 
  this->isJoyPublished_ = false;

  // initialize mode switch timer
  this->reset_autonomous_count();

  this->joy_msg_time_ = rclcpp::Clock().now();
  this->current_time_ = rclcpp::Clock().now();
  this->throttle_time_ = rclcpp::Clock().now();
  this->steering_time_ = rclcpp::Clock().now();

  // Range should be negative since 
  this->steer_range_ = this->min_steer_val_ - this->max_steer_val_;
  this->throttle_range_ = this->min_throttle_val_ - this->max_throttle_val_;

  // Set initial message states
  this->steer_servo_msg_.value = this->max_steer_val_ + 0.5*this->steer_range_;
  this->throttle_servo_msg_.value = this->max_throttle_val_;

  // set servo numbers
  this->steer_servo_msg_.servo = this->steer_servo_num_;
  this->throttle_servo_msg_.servo = this->drive_servo_num_;

  // Publish safety
  this->publish_safety_data();

  // Send servo setup request
  this->softsys_threads_.emplace_back(std::thread(&softsys_joy_node::send_servo_config_req, this));
  // this->send_servo_config_req();
  RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Constructor: softsys_joy_node");
}

softsys_joy_node::~softsys_joy_node()
{
  // Set servo states back to safe values and publish
  this->steer_servo_msg_.value = this->max_steer_val_ + 0.5*this->steer_range_;
  this->throttle_servo_msg_.value = this->max_throttle_val_;
  RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Destructor: STOPPING Servos");

  // check velocity to make sure it stops safely if possible
  for (int i = 0; i < 100; i++) {
    this->publish_servo_data();
  }

  // Join all threads before killing
  for (auto& th : this->softsys_threads_)
  {
    std::thread::id this_id = th.get_id();
    RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Joining Thread: #%s", this_id);
    th.join();
  }

  // need to send full stop command before closing node
  RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Destructor: softsys_joy_node");
}

void softsys_joy_node::controller_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // Receive joystick commands
  RCLCPP_DEBUG(rclcpp::get_logger("softsys_joy_node"), "Recieved Joystick Input");

  // update joystick message time
  this->isJoyPublished_ = true;
  this->joy_msg_time_ = rclcpp::Clock().now();

  // reset estop with left stick button
  if (this->eStop_) {
    if (msg->buttons[9] == 1) 
    {
      this->eStop_ = false;
    }
  }

  // MODE CONTROL
  if (this->canSwitchMode_)
  {
    // get button input, set isAuto, isAutoSteer and isAutoThrottle
    // Left shoulder button changes auto mode
    if (msg->buttons[5] == 1)
    {
      // Switch between auto and manual
      this->isAuto_ = !this->isAuto_;
      // If auto is now false, kill auto steer and throttle as well
      if (!this->isAuto_)
      {
        this->isAutoSteer_ = false;
        this->isAutoThrottle_ = false;
      }
      RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Set Auto Mode to: %s", this->isAuto_ ? "True" : "False");
      this->canSwitchMode_ = false;
    }

    if (msg->buttons[2] == 1 and !this->isAuto_)
    {
      // X button for auto throttle
      this->isAutoThrottle_ = !this->isAutoThrottle_;
      RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Set Auto Throttle to: %s", this->isAutoThrottle_ ? "True" : "False");
      this->canSwitchMode_ = false;
    }

    if (msg->buttons[3] == 1 and !this->isAuto_)
    {
      // Y button for auto steer
      this->isAutoSteer_ = !this->isAutoSteer_;
      RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Set Auto Steer to: %s", this->isAutoSteer_ ? "True" : "False");
      this->canSwitchMode_ = false;
    }
  }

  // MESSAGE UPDATES
  if ((!this->isAuto_ || !this->isAutoSteer_) && !this->eStop_)
  {
    // set controller to be steer value
    double controller_steer_pct = ((msg->axes[0]) + 1)/(2);
    RCLCPP_DEBUG(rclcpp::get_logger("softsys_joy_node"), "Controller steer pct: %f", controller_steer_pct);
    this->steer_servo_msg_.value = this->max_steer_val_ + this->steer_range_ * controller_steer_pct;
  }

  if ((!this->isAuto_ || !this->isAutoThrottle_) && !this->eStop_)
  {
    // set controller to be throttle value
    double controller_throttle_pct = ((-(msg->axes[5]) + 1))/2;

    // set controller reverse value as well
    double controller_throttle_pct_rev_ = ((-(msg->axes[2]) + 1))/2;

    RCLCPP_DEBUG(rclcpp::get_logger("softsys_joy_node"), "Controller throttle pct: %f, rev pct: %f", controller_throttle_pct, controller_throttle_pct_rev_);
    this->throttle_servo_msg_.value = this->max_throttle_val_ + this->throttle_range_ * (controller_throttle_pct - controller_throttle_pct_rev_); 
  }

}

void softsys_joy_node::throttle_callback(const softsys_msgs::msg::Throttle::SharedPtr msg)
{
  if (msg->throttle <= 1 && msg->throttle >= -1)
  {
    this->isRecievingAutoThrottle_ = true;
    this->throttle_time_ = rclcpp::Clock().now();
    // check if we are auto and autothrottle and not estop
    if (this->isAuto_ && this->isAutoThrottle_ && !this->eStop_)
    {
      this->throttle_servo_msg_.value = this->max_throttle_val_ + this->throttle_range_ * msg->throttle;
    }
  }
  else
  {
    this->isRecievingAutoThrottle_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("softsys_joy_node"), "Throttle out of bounds! %f", this->throttle_servo_msg_.value);
  }
}

void softsys_joy_node::steer_callback(const softsys_msgs::msg::Steer::SharedPtr msg) 
{
  if (msg->steer_angle <= 1 && msg->steer_angle >= -1)
  {
    this->isRecievingAutoSteer_ = true;
    this->steering_time_ = rclcpp::Clock().now();
    // check if we are auto and autosteer and not estop
    if (this->isAuto_ && this->isAutoSteer_ && !this->eStop_)
    {
      this->steer_servo_msg_.value = this->max_steer_val_ + 0.5*this->steer_range_ + msg->steer_angle*(this->steer_range_)*0.5; // 0 steering 
    }
  }
  else
  {
    this->isRecievingAutoSteer_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("softsys_joy_node"), "Steering out of bounds! %f", this->steer_servo_msg_.value);
  }
}

void softsys_joy_node::joy_timer_callback_()
{
  // publish new safety info
  this->publish_safety_data();

  // calculate current time differencen for message
  this->current_time_ = rclcpp::Clock().now();
  rclcpp::Duration diff_joy_time_ = this->current_time_ - this->joy_msg_time_;
  double timeout_dt_ = static_cast<double>(diff_joy_time_.seconds()) + static_cast<double>(diff_joy_time_.nanoseconds())*1e-9;

  // calculate current time differencen for message
  rclcpp::Duration diff_steer_time_ = this->current_time_ - this->steering_time_;
  double auto_steering_timeout_dt_ = static_cast<double>(diff_steer_time_.seconds()) + static_cast<double>(diff_steer_time_.nanoseconds())*1e-9;

  rclcpp::Duration diff_throttle_time_ = this->current_time_ - this->throttle_time_;
  double auto_throttle_timeout_dt_ = static_cast<double>(diff_throttle_time_.seconds()) + static_cast<double>(diff_throttle_time_.nanoseconds())*1e-9;

  RCLCPP_DEBUG(rclcpp::get_logger("softsys_joy_node"), "Controller Timeout: %f at timeout: %f", timeout_dt_, this->joy_timeout_);
  // Timeout checking
  if (timeout_dt_ > this->joy_timeout_ && this->isJoyPublished_)
  {
    this->eStop_ = true;
    RCLCPP_WARN(rclcpp::get_logger("softsys_joy_node"), "Controller connection timed out. Is it connected?");

    // Hard braking to stop car
    this->steer_servo_msg_.value = this->max_steer_val_;
    this->throttle_servo_msg_.value = this->max_throttle_val_;

    this->publish_servo_data();
    return;
  }

  // Check autonomous timeouts
  if(auto_steering_timeout_dt_ > this->auto_timeout_) {
    this->isRecievingAutoSteer_ = false;
  }

  if (auto_throttle_timeout_dt_ > this->auto_timeout_) {
    this->isRecievingAutoThrottle_ = false;
  }

  // if all is well, we publish servo array message with current data, whether auto or controller based
  if (!this->eStop_)
  {
    this->publish_servo_data();
  }

  // keep track of mode switching counter
  if (!this->canSwitchMode_)
  {
    this->autonomous_reset_count_--;
    if (this->autonomous_reset_count_ < 1)
    {
      this->canSwitchMode_ = true;
      this->reset_autonomous_count();
    }
  }

}

void softsys_joy_node::publish_servo_data()
{
  RCLCPP_WARN_ONCE(rclcpp::get_logger("softsys_joy_node"), "Publishing to servo!");
  this->servo_array_msg_.servos.emplace_back(this->steer_servo_msg_);
  this->servo_array_msg_.servos.emplace_back(this->throttle_servo_msg_);
  this->i2cpwm_pub_->publish(this->servo_array_msg_);
  this->servo_array_msg_.servos.clear();
}


void softsys_joy_node::reset_autonomous_count()
{
    this->autonomous_reset_count_ = (int)(this->autonomous_switching_delay_ / this->servo_update_rate_);
    if (this->autonomous_reset_count_ < 1) { this->autonomous_reset_count_ = 1; }
}

void softsys_joy_node::send_servo_config_req()
{
  RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Service request preparing!");

  auto servo_config_req_ = std::make_shared<i2cpwm_ros2_msgs::srv::ServosConfig::Request>();

  i2cpwm_ros2_msgs::msg::ServoConfig steer_servo_conf_;
  i2cpwm_ros2_msgs::msg::ServoConfig throttle_servo_conf_;

  steer_servo_conf_.servo = this->steer_servo_num_;
  steer_servo_conf_.center = this->max_steer_val_;
  steer_servo_conf_.range = this->max_steer_val_ - this->min_steer_val_;
  steer_servo_conf_.direction = -1;

  throttle_servo_conf_.servo = this->drive_servo_num_;
  throttle_servo_conf_.center = this->max_throttle_val_;
  throttle_servo_conf_.range = this->max_throttle_val_ - this->min_throttle_val_;
  throttle_servo_conf_.direction = -1;

  servo_config_req_->servos.emplace_back(throttle_servo_conf_);
  servo_config_req_->servos.emplace_back(steer_servo_conf_);

  // check if service is running
  while (!this->i2cpwm_srv_client_->wait_for_service(3s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("softsys_joy_node"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Service not available, waiting 3 seconds again...");
  }
  
  // Wait for the result.
  bool config_complete = false;
  RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Waiting for service response...");
  using ServiceResponseFuture = rclcpp::Client<i2cpwm_ros2_msgs::srv::ServosConfig>::SharedFuture;
  auto response_received_callback = [this, &config_complete](ServiceResponseFuture future) {
    RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Got config result: %d", future.get()->error);
    config_complete = true;
  };
  auto servo_config_result = this->i2cpwm_srv_client_->async_send_request(servo_config_req_, response_received_callback);
  servo_config_result.wait_for(5s);
  std::this_thread::sleep_for(200ms);
  if (!config_complete || servo_config_result.get() == NULL)
    RCLCPP_INFO(rclcpp::get_logger("softsys_joy_node"), "Timeout reached on config service!");
}

void softsys_joy_node::publish_safety_data()
{
  // fill message
  this->safety_msg_.header.stamp = rclcpp::Clock().now();
  this->safety_msg_.is_auto = this->isAuto_;
  this->safety_msg_.is_auto_steer = this->isAutoSteer_;
  this->safety_msg_.is_auto_throttle = this->isAutoThrottle_;
  this->safety_msg_.is_emergency_stop = this->eStop_;
  this->safety_msg_.is_recieving_joystick = this->isJoyPublished_;
  this->safety_msg_.is_recieving_auto_steer = this->isRecievingAutoSteer_;
  this->safety_msg_.is_recieving_auto_throttle = this->isRecievingAutoThrottle_;
  this->safety_pub_->publish(this->safety_msg_);
}


// Main Loop
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<softsys_joy_node>());
  rclcpp::shutdown();
  return 0;
}