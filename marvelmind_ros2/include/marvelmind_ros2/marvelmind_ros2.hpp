// C++ STL
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <fcntl.h>
#include <thread>
using namespace std::chrono_literals;

extern "C"
{
#include "marvelmind_hedge.h"
}

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

// Needed external messages
#include "marvelmind_ros2_msgs/msg/hedge_position.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position_addressed.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_position_angle.hpp"

#include "marvelmind_ros2_msgs/msg/hedge_imu_raw.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_imu_fusion.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_telemetry.hpp"
#include "marvelmind_ros2_msgs/msg/hedge_quality.hpp"

#include "marvelmind_ros2_msgs/msg/beacon_distance.hpp"
#include "marvelmind_ros2_msgs/msg/beacon_position_addressed.hpp"

#include "marvelmind_ros2_msgs/msg/marvelmind_waypoint.hpp"

// global vars
// global semaphore to avoid class scope issues
static sem_t *sem;

class marvelmind_ros2 : public rclcpp::Node
{
public:
    marvelmind_ros2();
    ~marvelmind_ros2();

private:
    // ROS2 Setup
    // Publishers
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgePositionAngle>::SharedPtr hedge_pos_ang_publisher;
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgePositionAddressed>::SharedPtr hedge_pos_publisher;
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgePosition>::SharedPtr hedge_pos_noaddress_publisher;
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::BeaconPositionAddressed>::SharedPtr beacons_pos_publisher;
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgeImuRaw>::SharedPtr hedge_imu_raw_publisher;
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgeImuFusion>::SharedPtr hedge_imu_fusion_publisher;
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::BeaconDistance>::SharedPtr beacon_distance_publisher;
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgeTelemetry>::SharedPtr hedge_telemetry_publisher;
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::HedgeQuality>::SharedPtr hedge_quality_publisher;
    rclcpp::Publisher<marvelmind_ros2_msgs::msg::MarvelmindWaypoint>::SharedPtr marvelmind_waypoint_publisher;

    // Timer to execute publications so we can easily control rate
    rclcpp::TimerBase::SharedPtr marvelmind_ros2_pub_timer;

    // Function prototypes
    int hedgeReceivePrepare();
    bool hedgeReceiveCheck(void);
    bool beaconReceiveCheck(void);
    bool hedgeIMURawReceiveCheck(void);
    bool hedgeIMUFusionReceiveCheck(void);
    void getRawDistance(uint8_t index);
    bool hedgeTelemetryUpdateCheck(void);
    bool hedgeQualityUpdateCheck(void);
    bool marvelmindWaypointUpdateCheck(void);
    void quatToEul(marvelmind_ros2_msgs::msg::HedgeImuFusion &msg);
    void publishTimerCallback();

    // Variables
    // Topic names
    std::string hedgehog_pos_topic;
    std::string hedgehog_pos_addressed_topic;
    std::string hedgehog_pos_angle_topic;
    std::string hedgehog_imu_raw_topic;
    std::string hedgehog_imu_fusion_topic;
    std::string hedgehog_telemetry_topic;
    std::string hedgehog_quality_topic;
    std::string beacon_raw_distance_topic;
    std::string beacon_pos_addressed_topic;
    std::string marvelmind_waypoint_topic;

    // Config variables
    std::string data_input_semaphore_name;
    uint tty_baudrate;
    std::string tty_filename;
    int publish_rate_in_hz;
    std::chrono::milliseconds publish_rate_ms;

    // Marvelmind data
    struct MarvelmindHedge *hedge = NULL;
    uint32_t hedge_timestamp_prev = 0;
    struct timespec ts;
    uint8_t beaconReadIterations;

    // setup empty messages for sending
    marvelmind_ros2_msgs::msg::HedgePosition hedge_pos_noaddress_msg;      // hedge coordinates message (old version without address) for publishing to ROS topic
    marvelmind_ros2_msgs::msg::HedgePositionAddressed hedge_pos_msg;       // hedge coordinates message for publishing to ROS topic
    marvelmind_ros2_msgs::msg::HedgePositionAngle hedge_pos_ang_msg;       // hedge coordinates and angle message for publishing to ROS topic
    marvelmind_ros2_msgs::msg::BeaconPositionAddressed beacon_pos_msg;     // stationary beacon coordinates message for publishing to ROS topic
    marvelmind_ros2_msgs::msg::HedgeImuRaw hedge_imu_raw_msg;              // raw IMU data message for publishing to ROS topic
    marvelmind_ros2_msgs::msg::HedgeImuFusion hedge_imu_fusion_msg;        // IMU fusion data message for publishing to ROS topic
    marvelmind_ros2_msgs::msg::BeaconDistance beacon_raw_distance_msg;     // Raw distance message for publishing to ROS topic
    marvelmind_ros2_msgs::msg::HedgeTelemetry hedge_telemetry_msg;         // Telemetry message for publishing to ROS topic
    marvelmind_ros2_msgs::msg::HedgeQuality hedge_quality_msg;             // Quality message for publishing to ROS topic
    marvelmind_ros2_msgs::msg::MarvelmindWaypoint marvelmind_waypoint_msg; // Waypoint message for publishing to ROS topic
};
