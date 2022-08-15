#include <marvelmind_ros2/marvelmind_ros2.hpp>

marvelmind_ros2::marvelmind_ros2() : rclcpp::Node("marvelmind_ros2")
{
  static const rclcpp::Logger hedgehog_logger = rclcpp::get_logger("hedgehog_logger");
  RCLCPP_INFO(rclcpp::get_logger("hedgehog_logger"), "Constructor: marvelmind_ros2 pre-init");

  // Declare params found in config
  // Topics
  this->declare_parameter("hedgehog_pos_topic", "hedgehog_pos");
  this->declare_parameter("hedgehog_pos_addressed_topic", "hedgehog_pos_addressed");
  this->declare_parameter("hedgehog_pos_angle_topic", "hedgehog_pos_ang");
  this->declare_parameter("beacon_pos_addressed_topic", "beacons_pos_addressed");
  this->declare_parameter("hedgehog_imu_raw_topic", "hedgehog_imu_raw");
  this->declare_parameter("hedgehog_imu_fusion_topic", "hedgehog_imu_fusion");
  this->declare_parameter("beacon_raw_distance_topic", "beacon_raw_distance");
  this->declare_parameter("hedgehog_telemetry_topic", "hedgehog_telemetry");
  this->declare_parameter("hedgehog_quality_topic", "hedgehog_quality");
  this->declare_parameter("marvelmind_waypoint_topic", "marvelmind_waypoint");

  // Other Params
  this->declare_parameter("data_input_semaphore_name", "/data_input_semaphore");
  this->declare_parameter("publish_rate_in_hz", 1);
  this->declare_parameter("tty_baudrate", 9600);
  this->declare_parameter("tty_filename", "/dev/ttyACM0");

  // Startup
  //  Get all params
  this->hedgehog_pos_topic = this->get_parameter("hedgehog_pos_topic").as_string();
  this->hedgehog_pos_addressed_topic = this->get_parameter("hedgehog_pos_addressed_topic").as_string();
  this->hedgehog_pos_angle_topic = this->get_parameter("hedgehog_pos_angle_topic").as_string();
  this->hedgehog_imu_raw_topic = this->get_parameter("hedgehog_imu_raw_topic").as_string();
  this->hedgehog_imu_fusion_topic = this->get_parameter("hedgehog_imu_fusion_topic").as_string();
  this->hedgehog_telemetry_topic = this->get_parameter("hedgehog_telemetry_topic").as_string();
  this->hedgehog_quality_topic = this->get_parameter("hedgehog_quality_topic").as_string();
  this->beacon_raw_distance_topic = this->get_parameter("beacon_raw_distance_topic").as_string();
  this->beacon_pos_addressed_topic = this->get_parameter("beacon_pos_addressed_topic").as_string();
  this->marvelmind_waypoint_topic = this->get_parameter("marvelmind_waypoint_topic").as_string();

  this->data_input_semaphore_name = this->get_parameter("data_input_semaphore_name").as_string();
  this->publish_rate_in_hz = this->get_parameter("publish_rate_in_hz").as_int();
  this->tty_baudrate = this->get_parameter("tty_baudrate").as_int();
  this->tty_filename = this->get_parameter("tty_filename").as_string();

  // set publishers
  this->hedge_pos_ang_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::HedgePositionAngle>(this->hedgehog_pos_angle_topic, 20);

  this->hedge_pos_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::HedgePositionAddressed>(this->hedgehog_pos_addressed_topic, 20);

  this->hedge_pos_noaddress_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::HedgePosition>(this->hedgehog_pos_topic, 20);

  this->beacons_pos_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::BeaconPositionAddressed>(this->beacon_pos_addressed_topic, 20);

  this->hedge_imu_raw_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::HedgeImuRaw>(this->hedgehog_imu_raw_topic, 20);

  this->hedge_imu_fusion_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::HedgeImuFusion>(this->hedgehog_imu_fusion_topic, 20);

  this->beacon_distance_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::BeaconDistance>(this->beacon_raw_distance_topic, 20);

  this->hedge_telemetry_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::HedgeTelemetry>(this->hedgehog_telemetry_topic, 20);

  this->hedge_quality_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::HedgeQuality>(this->hedgehog_quality_topic, 20);

  this->marvelmind_waypoint_publisher = this->create_publisher<marvelmind_ros2_msgs::msg::MarvelmindWaypoint>(this->marvelmind_waypoint_topic, 20);

  // do setup
  sem = sem_open(this->data_input_semaphore_name.c_str(), O_CREAT, 0777, 0);
  this->hedgeReceivePrepare();

  // set default message values:
  this->hedge_pos_ang_msg.address = 0;
  this->hedge_pos_ang_msg.timestamp_ms = 0;
  this->hedge_pos_ang_msg.x_m = 0.0;
  this->hedge_pos_ang_msg.y_m = 0.0;
  this->hedge_pos_ang_msg.z_m = 0.0;
  this->hedge_pos_ang_msg.flags = (1 << 0); // 'data not available' flag
  this->hedge_pos_ang_msg.angle = 0.0;

  this->hedge_pos_msg.address = 0;
  this->hedge_pos_msg.timestamp_ms = 0;
  this->hedge_pos_msg.x_m = 0.0;
  this->hedge_pos_msg.y_m = 0.0;
  this->hedge_pos_msg.z_m = 0.0;
  this->hedge_pos_msg.flags = (1 << 0); // 'data not available' flag

  this->hedge_pos_noaddress_msg.timestamp_ms = 0;
  this->hedge_pos_noaddress_msg.x_m = 0.0;
  this->hedge_pos_noaddress_msg.y_m = 0.0;
  this->hedge_pos_noaddress_msg.z_m = 0.0;
  this->hedge_pos_noaddress_msg.flags = (1 << 0); // 'data not available' flag

  this->beacon_pos_msg.address = 0;
  this->beacon_pos_msg.x_m = 0.0;
  this->beacon_pos_msg.y_m = 0.0;
  this->beacon_pos_msg.z_m = 0.0;

  this->publish_rate_ms = std::chrono::milliseconds((int)round(1000.0 / this->publish_rate_in_hz));
  this->marvelmind_ros2_pub_timer = this->create_wall_timer(this->publish_rate_ms, std::bind(&marvelmind_ros2::publishTimerCallback, this));

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Pub rate ms: %li", this->publish_rate_ms.count());
}

marvelmind_ros2::~marvelmind_ros2()
{
  RCLCPP_INFO(rclcpp::get_logger("hedgehog_logger"), "Destructor starting marvelmind_ros2...");

  if (this->hedge != NULL)
  {
    stopMarvelmindHedge(this->hedge);
    destroyMarvelmindHedge(this->hedge);
  }

  sem_close(sem);
  RCLCPP_INFO(rclcpp::get_logger("hedgehog_logger"), "Destructor: marvelmind_ros2");
}

// Global semaphore callback to avoid issues with class scope
void semCallback()
{
  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Using sem callback...");
  sem_post(sem);
}

int marvelmind_ros2::hedgeReceivePrepare()
{
  // port name comes from config file now instead of command line, same with baud rate
  const char *ttyFileName = this->tty_filename.c_str();
  uint32_t baudRate = this->tty_baudrate;
  RCLCPP_INFO(rclcpp::get_logger("hedgehog_logger"), "Prepare: tty: %s and baud: %d", this->tty_filename.c_str(), this->tty_baudrate);

  // Init
  this->hedge = createMarvelmindHedge();
  if (this->hedge == NULL)
  {
    RCLCPP_ERROR(rclcpp::get_logger("hedgehog_logger"), "Error: Unable to create MarvelmindHedge");
    return -1;
  }
  this->hedge->ttyFileName = ttyFileName;
  this->hedge->baudRate = baudRate;
  this->hedge->verbose = true; // show errors and warnings
  this->hedge->anyInputPacketCallback = semCallback;
  startMarvelmindHedge(this->hedge);
  RCLCPP_INFO(rclcpp::get_logger("hedgehog_logger"), "Hedgehog is running!");

  return 0;
}

bool marvelmind_ros2::hedgeReceiveCheck(void)
{
  if (this->hedge->haveNewValues_)
  {
    struct PositionValue position;
    getPositionFromMarvelmindHedge(this->hedge, &position);

    this->hedge_pos_msg.address = position.address;
    this->hedge_pos_ang_msg.address = position.address;

    this->hedge_pos_msg.flags = position.flags;
    this->hedge_pos_noaddress_msg.flags = position.flags;
    this->hedge_pos_ang_msg.flags = position.flags;
    if (this->hedge_pos_msg.flags & (1 << 1)) // flag of timestamp format
    {
      this->hedge_pos_msg.timestamp_ms = position.timestamp; // msec
      this->hedge_pos_noaddress_msg.timestamp_ms = position.timestamp;
    }
    else
    {
      this->hedge_pos_msg.timestamp_ms = position.timestamp * 15.625; // alpha-cycles ==> msec
      this->hedge_pos_noaddress_msg.timestamp_ms = position.timestamp * 15.625;
    }
    this->hedge_pos_ang_msg.timestamp_ms = position.timestamp;

    this->hedge_pos_msg.x_m = position.x / 1000.0;
    this->hedge_pos_msg.y_m = position.y / 1000.0;
    this->hedge_pos_msg.z_m = position.z / 1000.0;

    this->hedge_pos_noaddress_msg.x_m = position.x / 1000.0;
    this->hedge_pos_noaddress_msg.y_m = position.y / 1000.0;
    this->hedge_pos_noaddress_msg.z_m = position.z / 1000.0;

    this->hedge_pos_ang_msg.x_m = position.x / 1000.0;
    this->hedge_pos_ang_msg.y_m = position.y / 1000.0;
    this->hedge_pos_ang_msg.z_m = position.z / 1000.0;

    this->hedge_pos_ang_msg.angle = position.angle;

    this->hedge->haveNewValues_ = false;

    return true;
  }
  return false;
}

bool marvelmind_ros2::beaconReceiveCheck(void)
{
  uint8_t i;
  struct StationaryBeaconsPositions positions;
  struct StationaryBeaconPosition *bp = NULL;
  bool foundUpd = false;
  uint8_t n;

  getStationaryBeaconsPositionsFromMarvelmindHedge(this->hedge, &positions);
  n = positions.numBeacons;
  if (n == 0)
    return false;

  for (i = 0; i < n; i++)
  {
    bp = &positions.beacons[i];
    if (bp->updatedForMsg)
    {
      clearStationaryBeaconUpdatedFlag(this->hedge, bp->address);
      foundUpd = true;
      break;
    }
  }
  if (!foundUpd)
    return false;
  if (bp == NULL)
    return false;

  this->beacon_pos_msg.address = bp->address;
  this->beacon_pos_msg.x_m = bp->x / 1000.0;
  this->beacon_pos_msg.y_m = bp->y / 1000.0;
  this->beacon_pos_msg.z_m = bp->z / 1000.0;

  return true;
}

bool marvelmind_ros2::hedgeIMURawReceiveCheck(void)
{
  if (!this->hedge->rawIMU.updated)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Hedgehog rawIMU not updated!");
    return false;
  }

  this->hedge_imu_raw_msg.acc_x = this->hedge->rawIMU.acc_x;
  this->hedge_imu_raw_msg.acc_y = this->hedge->rawIMU.acc_y;
  this->hedge_imu_raw_msg.acc_z = this->hedge->rawIMU.acc_z;

  this->hedge_imu_raw_msg.gyro_x = this->hedge->rawIMU.gyro_x;
  this->hedge_imu_raw_msg.gyro_y = this->hedge->rawIMU.gyro_y;
  this->hedge_imu_raw_msg.gyro_z = this->hedge->rawIMU.gyro_z;

  this->hedge_imu_raw_msg.compass_x = this->hedge->rawIMU.compass_x;
  this->hedge_imu_raw_msg.compass_y = this->hedge->rawIMU.compass_y;
  this->hedge_imu_raw_msg.compass_z = this->hedge->rawIMU.compass_z;

  this->hedge_imu_raw_msg.timestamp_ms = this->hedge->rawIMU.timestamp;

  this->hedge->rawIMU.updated = false;

  return true;
}

bool marvelmind_ros2::hedgeIMUFusionReceiveCheck(void)
{
  if (!this->hedge->fusionIMU.updated)
    return false;

  this->hedge_imu_fusion_msg.x_m = this->hedge->fusionIMU.x / 1000.0;
  this->hedge_imu_fusion_msg.y_m = this->hedge->fusionIMU.y / 1000.0;
  this->hedge_imu_fusion_msg.z_m = this->hedge->fusionIMU.z / 1000.0;

  this->hedge_imu_fusion_msg.qw = this->hedge->fusionIMU.qw / 10000.0;
  this->hedge_imu_fusion_msg.qx = this->hedge->fusionIMU.qx / 10000.0;
  this->hedge_imu_fusion_msg.qy = this->hedge->fusionIMU.qy / 10000.0;
  this->hedge_imu_fusion_msg.qz = this->hedge->fusionIMU.qz / 10000.0;

  quatToEul(this->hedge_imu_fusion_msg);

  this->hedge_imu_fusion_msg.vx = this->hedge->fusionIMU.vx / 1000.0;
  this->hedge_imu_fusion_msg.vy = this->hedge->fusionIMU.vy / 1000.0;
  this->hedge_imu_fusion_msg.vz = this->hedge->fusionIMU.vz / 1000.0;

  this->hedge_imu_fusion_msg.ax = this->hedge->fusionIMU.ax / 1000.0;
  this->hedge_imu_fusion_msg.ay = this->hedge->fusionIMU.ay / 1000.0;
  this->hedge_imu_fusion_msg.az = this->hedge->fusionIMU.az / 1000.0;

  this->hedge_imu_fusion_msg.timestamp_ms = this->hedge->fusionIMU.timestamp;

  this->hedge->fusionIMU.updated = false;

  return true;
}

void marvelmind_ros2::getRawDistance(uint8_t index)
{
  this->beacon_raw_distance_msg.address_hedge = this->hedge->rawDistances.address_hedge;
  this->beacon_raw_distance_msg.address_beacon = this->hedge->rawDistances.distances[index].address_beacon;
  this->beacon_raw_distance_msg.distance_m = this->hedge->rawDistances.distances[index].distance / 1000.0;
}

bool marvelmind_ros2::hedgeTelemetryUpdateCheck(void)
{
  if (!this->hedge->telemetry.updated)
    return false;

  this->hedge_telemetry_msg.battery_voltage = this->hedge->telemetry.vbat_mv / 1000.0;
  this->hedge_telemetry_msg.rssi_dbm = this->hedge->telemetry.rssi_dbm;

  this->hedge->telemetry.updated = false;
  return true;
}

bool marvelmind_ros2::hedgeQualityUpdateCheck(void)
{
  if (!this->hedge->quality.updated)
    return false;

  this->hedge_quality_msg.address = this->hedge->quality.address;
  this->hedge_quality_msg.quality_percents = this->hedge->quality.quality_per;

  this->hedge->quality.updated = false;
  return true;
}

bool marvelmind_ros2::marvelmindWaypointUpdateCheck(void)
{
  uint8_t i, n;
  uint8_t nUpdated;

  if (!this->hedge->waypoints.updated)
    return false;

  nUpdated = 0;
  n = this->hedge->waypoints.numItems;
  for (i = 0; i < n; i++)
  {
    if (!this->hedge->waypoints.items[i].updated)
      continue;

    nUpdated++;
    if (nUpdated == 1)
    {
      this->marvelmind_waypoint_msg.total_items = n;
      this->marvelmind_waypoint_msg.item_index = i;

      this->marvelmind_waypoint_msg.movement_type = this->hedge->waypoints.items[i].movementType;
      this->marvelmind_waypoint_msg.param1 = this->hedge->waypoints.items[i].param1;
      this->marvelmind_waypoint_msg.param2 = this->hedge->waypoints.items[i].param2;
      this->marvelmind_waypoint_msg.param3 = this->hedge->waypoints.items[i].param3;

      this->hedge->waypoints.items[i].updated = false;
    }
  }

  if (nUpdated == 1)
  {
    this->hedge->waypoints.updated = false;
  }
  return (nUpdated > 0);
}

// #define _USE_MATH_DEFINES

void marvelmind_ros2::quatToEul(marvelmind_ros2_msgs::msg::HedgeImuFusion &msg)
{
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (msg.qw * msg.qx + msg.qy * msg.qz);
  double cosr_cosp = 1 - 2 * (msg.qx * msg.qx + msg.qy * msg.qy);
  msg.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (msg.qw * msg.qy - msg.qz * msg.qx);
  if (std::abs(sinp) >= 1)
    msg.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    msg.pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (msg.qw * msg.qz + msg.qx * msg.qy);
  double cosy_cosp = 1 - 2 * (msg.qy * msg.qy + msg.qz * msg.qz);
  msg.yaw = std::atan2(siny_cosp, cosy_cosp);
}

void marvelmind_ros2::publishTimerCallback()
{
  // Timer for firing publishers at defined update rate from config file
  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Callback!");
  if (this->hedge->terminationRequired)
  {
    RCLCPP_WARN(rclcpp::get_logger("hedgehog_logger"), "Termination Required");
  }

  if (clock_gettime(CLOCK_REALTIME, &this->ts) == -1)
  {
    RCLCPP_WARN(rclcpp::get_logger("hedgehog_logger"), "Clock Get Time Error");
    return;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Starting Checks...");
  this->ts.tv_sec += 2;
  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Starting Checks 2...");
  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Timespec: %11f", (double)this->ts.tv_sec);

  // causes pause until semaphore can move on, always waits for 2 seconds
  sem_timedwait(sem, &ts);

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Do Hedge Receive Check!");
  if (this->hedgeReceiveCheck())
  {
    // hedgehog data received
    RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Address: %d, timestamp: %d, %d, X=%.3f  Y= %.3f  Z=%.3f  Angle: %.1f  flags=%d",
                 (int)this->hedge_pos_ang_msg.address,
                 (int)this->hedge_pos_ang_msg.timestamp_ms,
                 (int)(this->hedge_pos_ang_msg.timestamp_ms - this->hedge_timestamp_prev),
                 (float)this->hedge_pos_ang_msg.x_m, (float)this->hedge_pos_ang_msg.y_m, (float)this->hedge_pos_ang_msg.z_m,
                 (float)this->hedge_pos_ang_msg.angle,
                 (int)this->hedge_pos_msg.flags);
    this->hedge_pos_ang_publisher->publish(this->hedge_pos_ang_msg);
    this->hedge_pos_publisher->publish(this->hedge_pos_msg);
    this->hedge_pos_noaddress_publisher->publish(this->hedge_pos_noaddress_msg);

    this->hedge_timestamp_prev = this->hedge_pos_ang_msg.timestamp_ms;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Do Beacon recieve check!");
  while (this->beaconReceiveCheck())
  {
    // stationary beacons data received
    RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Stationary beacon: Address: %d, X=%.3f  Y= %.3f  Z=%.3f",
                 (int)this->beacon_pos_msg.address,
                 (float)this->beacon_pos_msg.x_m, (float)this->beacon_pos_msg.y_m, (float)this->beacon_pos_msg.z_m);
    this->beacons_pos_publisher->publish(this->beacon_pos_msg);

    this->beaconReadIterations++;
    if (this->beaconReadIterations > 4)
      break;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Do hedge imu raw check!");
  if (this->hedgeIMURawReceiveCheck())
  {
    RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Raw IMU: Timestamp: %08d, aX=%05d aY=%05d aZ=%05d  gX=%05d gY=%05d gZ=%05d  cX=%05d cY=%05d cZ=%05d",
                 (int)this->hedge_imu_raw_msg.timestamp_ms,
                 (int)this->hedge_imu_raw_msg.acc_x, (int)this->hedge_imu_raw_msg.acc_y, (int)this->hedge_imu_raw_msg.acc_z,
                 (int)this->hedge_imu_raw_msg.gyro_x, (int)this->hedge_imu_raw_msg.gyro_y, (int)this->hedge_imu_raw_msg.gyro_z,
                 (int)this->hedge_imu_raw_msg.compass_x, (int)this->hedge_imu_raw_msg.compass_y, (int)this->hedge_imu_raw_msg.compass_z);
    this->hedge_imu_raw_publisher->publish(this->hedge_imu_raw_msg);
  }

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Do hedge IMU fusion check!");
  if (this->hedgeIMUFusionReceiveCheck())
  {
    RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "IMU fusion: Timestamp: %08d, X=%.3f  Y= %.3f  Z=%.3f  q=%.3f,%.3f,%.3f,%.3f v=%.3f,%.3f,%.3f  a=%.3f,%.3f,%.3f",
                 (int)this->hedge_imu_fusion_msg.timestamp_ms,
                 (float)this->hedge_imu_fusion_msg.x_m, (float)this->hedge_imu_fusion_msg.y_m, (float)this->hedge_imu_fusion_msg.z_m,
                 (float)this->hedge_imu_fusion_msg.qw, (float)this->hedge_imu_fusion_msg.qx, (float)this->hedge_imu_fusion_msg.qy, (float)this->hedge_imu_fusion_msg.qz,
                 (float)this->hedge_imu_fusion_msg.vx, (float)this->hedge_imu_fusion_msg.vy, (float)this->hedge_imu_fusion_msg.vz,
                 (float)this->hedge_imu_fusion_msg.ax, (float)this->hedge_imu_fusion_msg.ay, (float)this->hedge_imu_fusion_msg.az);
    this->hedge_imu_fusion_publisher->publish(this->hedge_imu_fusion_msg);
  }

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Do hedge raw distance update!");
  if (this->hedge->rawDistances.updated)
  {
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
      this->getRawDistance(i);
      if (this->beacon_raw_distance_msg.address_beacon != 0)
      {
        RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Raw distance: %02d ==> %02d,  Distance= %.3f ",
                     (int)this->beacon_raw_distance_msg.address_hedge,
                     (int)this->beacon_raw_distance_msg.address_beacon,
                     (float)this->beacon_raw_distance_msg.distance_m);
        this->beacon_distance_publisher->publish(this->beacon_raw_distance_msg);
      }
    }
    this->hedge->rawDistances.updated = false;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Do hedge telem check!");
  if (this->hedgeTelemetryUpdateCheck())
  {
    RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Vbat= %.3f V, RSSI= %02d ",
                 (float)this->hedge_telemetry_msg.battery_voltage,
                 (int)this->hedge_telemetry_msg.rssi_dbm);
    this->hedge_telemetry_publisher->publish(this->hedge_telemetry_msg);
  }

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Do hedge quality check!");
  if (this->hedgeQualityUpdateCheck())
  {
    RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Quality: Address= %d,  Quality= %02d %% ",
                 (int)this->hedge_quality_msg.address,
                 (int)this->hedge_quality_msg.quality_percents);
    this->hedge_quality_publisher->publish(this->hedge_quality_msg);
  }

  RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Do marvel waypoint check!");

  if (this->marvelmindWaypointUpdateCheck())
  {
    int n = this->marvelmind_waypoint_msg.item_index + 1;
    RCLCPP_DEBUG(rclcpp::get_logger("hedgehog_logger"), "Waypoint %03d/%03d: Type= %03d,  Param1= %05d, Param2= %05d, Param3= %05d ",
                 (int)n,
                 (int)this->marvelmind_waypoint_msg.total_items, this->marvelmind_waypoint_msg.movement_type,
                 this->marvelmind_waypoint_msg.param1, this->marvelmind_waypoint_msg.param2, this->marvelmind_waypoint_msg.param3);
    this->marvelmind_waypoint_publisher->publish(this->marvelmind_waypoint_msg);
  }

} // end timer callback

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // create and spin marvelmind ros2 node
  auto marvelmind_ros2_node = std::make_shared<marvelmind_ros2>();
  rclcpp::spin(marvelmind_ros2_node);
  rclcpp::shutdown();
  return 0;
}
