#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp" 
#include "dynamixel_sdk/dynamixel_sdk.h"

// Define addresses and lengths (Feedback registers)
#define ADDR_BULK_START         126   // Starts at Present Current
#define BULK_DATA_LENGTH        10    // Covers Present Current (2 bytes), Present Velocity (4 bytes), and Present Position (4 bytes)

#define ADDR_PRESENT_CURRENT    126   // 2 bytes
#define ADDR_PRESENT_VELOCITY   128   // 4 bytes
#define ADDR_PRESENT_POSITION   132   // 4 bytes

// Goal Current address (for commanding)
#define ADDR_GOAL_CURRENT       102   // 2 bytes

// Operating Mode address and default value
#define ADDR_OPERATING_MODE     11    // Operating Mode (EEPROM Area)

// PID Parameter addresses for Velocity and Position
#define ADDR_VELOCITY_I_GAIN    76    // 2 bytes
#define ADDR_VELOCITY_P_GAIN    78    // 2 bytes
#define ADDR_POSITION_D_GAIN    80    // 2 bytes
#define ADDR_POSITION_I_GAIN    82    // 2 bytes
#define ADDR_POSITION_P_GAIN    84    // 2 bytes
#define ADDR_FEEDFORWARD_2ND_GAIN 88  // 2 bytes
#define ADDR_FEEDFORWARD_1ST_GAIN 90  // 2 bytes

// Additional registers from EEPROM (example values)
#define ADDR_TEMPERATURE_LIMIT  31    // 1 byte
#define ADDR_MAX_VOLTAGE_LIMIT  32    // 2 bytes
#define ADDR_MIN_VOLTAGE_LIMIT  34    // 2 bytes
#define ADDR_PWM_LIMIT          36    // 2 bytes
#define ADDR_CURRENT_LIMIT      38    // 2 bytes
#define ADDR_VELOCITY_LIMIT     44    // 4 bytes
#define ADDR_MAX_POSITION_LIMIT 48    // 4 bytes
#define ADDR_MIN_POSITION_LIMIT 52    // 4 bytes

// Device settings
#define PROTOCOL_VERSION 2.0
// #define BAUDRATE 1000000
// #define DEVICENAME "/dev/ttyUSB0"

std::string DEVICENAME;
int BAUDRATE;

class MultiMotorControlNode : public rclcpp::Node
{
public:
  MultiMotorControlNode() : Node("multi_motor_control_node")
  {

    // Load global default parameters
    this->declare_parameter<std::vector<int>>("motor_ids", {12, 41, 42});
    this->declare_parameter<double>("default_current_factor", 2.69);
    this->declare_parameter<double>("default_velocity_factor", 0.229);
    this->declare_parameter<double>("default_position_full_scale", 4095.0);
    this->declare_parameter<int>("default_operating_mode", 1);
    this->declare_parameter<int>("default_temperature_limit", 80);
    this->declare_parameter<int>("default_max_voltage_limit", 160);
    this->declare_parameter<int>("default_min_voltage_limit", 95);
    this->declare_parameter<int>("default_pwm_limit", 885);
    this->declare_parameter<int>("default_current_limit", 1193);
    this->declare_parameter<int>("default_velocity_limit", 200);
    this->declare_parameter<int>("default_max_position_limit", 4095);
    this->declare_parameter<int>("default_min_position_limit", 0);
    this->declare_parameter<int>("default_velocity_i_gain", 1920);
    this->declare_parameter<int>("default_velocity_p_gain", 100);
    this->declare_parameter<int>("default_position_d_gain", 0);
    this->declare_parameter<int>("default_position_i_gain", 0);
    this->declare_parameter<int>("default_position_p_gain", 800);
    this->declare_parameter<int>("default_feedforward_2nd_gain", 0);
    this->declare_parameter<int>("default_feedforward_1st_gain", 0);

    // motor_ids_ = this->get_parameter("motor_ids").as_integer_array();
    auto motor_ids_64 = this->get_parameter("motor_ids").as_integer_array();
    motor_ids_ = std::vector<int>(motor_ids_64.begin(), motor_ids_64.end());

    // Load per-motor configuration
    for (auto id : motor_ids_) {
      std::string ns = "motor_" + std::to_string(id);
      this->declare_parameter<double>(ns + ".current_factor", this->get_parameter("default_current_factor").as_double());
      this->declare_parameter<double>(ns + ".velocity_factor", this->get_parameter("default_velocity_factor").as_double());
      this->declare_parameter<double>(ns + ".position_full_scale", this->get_parameter("default_position_full_scale").as_double());
      this->declare_parameter<int>(ns + ".operating_mode", this->get_parameter("default_operating_mode").as_int());
      this->declare_parameter<int>(ns + ".temperature_limit", this->get_parameter("default_temperature_limit").as_int());
      this->declare_parameter<int>(ns + ".max_voltage_limit", this->get_parameter("default_max_voltage_limit").as_int());
      this->declare_parameter<int>(ns + ".min_voltage_limit", this->get_parameter("default_min_voltage_limit").as_int());
      this->declare_parameter<int>(ns + ".pwm_limit", this->get_parameter("default_pwm_limit").as_int());
      this->declare_parameter<int>(ns + ".current_limit", this->get_parameter("default_current_limit").as_int());
      this->declare_parameter<int>(ns + ".velocity_limit", this->get_parameter("default_velocity_limit").as_int());
      this->declare_parameter<int>(ns + ".max_position_limit", this->get_parameter("default_max_position_limit").as_int());
      this->declare_parameter<int>(ns + ".min_position_limit", this->get_parameter("default_min_position_limit").as_int());
      this->declare_parameter<int>(ns + ".velocity_i_gain", this->get_parameter("default_velocity_i_gain").as_int());
      this->declare_parameter<int>(ns + ".velocity_p_gain", this->get_parameter("default_velocity_p_gain").as_int());
      this->declare_parameter<int>(ns + ".position_d_gain", this->get_parameter("default_position_d_gain").as_int());
      this->declare_parameter<int>(ns + ".position_i_gain", this->get_parameter("default_position_i_gain").as_int());
      this->declare_parameter<int>(ns + ".position_p_gain", this->get_parameter("default_position_p_gain").as_int());
      this->declare_parameter<int>(ns + ".feedforward_2nd_gain", this->get_parameter("default_feedforward_2nd_gain").as_int());
      this->declare_parameter<int>(ns + ".feedforward_1st_gain", this->get_parameter("default_feedforward_1st_gain").as_int());

      // Save per-motor config in maps
      motor_current_factor_map_[id] = this->get_parameter(ns + ".current_factor").as_double();
      motor_velocity_factor_map_[id] = this->get_parameter(ns + ".velocity_factor").as_double();
      motor_position_full_scale_map_[id] = this->get_parameter(ns + ".position_full_scale").as_double();
      motor_operating_mode_map_[id] = this->get_parameter(ns + ".operating_mode").as_int();
      motor_temperature_limit_map_[id] = this->get_parameter(ns + ".temperature_limit").as_int();
      motor_max_voltage_limit_map_[id] = this->get_parameter(ns + ".max_voltage_limit").as_int();
      motor_min_voltage_limit_map_[id] = this->get_parameter(ns + ".min_voltage_limit").as_int();
      motor_pwm_limit_map_[id] = this->get_parameter(ns + ".pwm_limit").as_int();
      motor_current_limit_map_[id] = this->get_parameter(ns + ".current_limit").as_int();
      motor_velocity_limit_map_[id] = this->get_parameter(ns + ".velocity_limit").as_int();
      motor_max_position_limit_map_[id] = this->get_parameter(ns + ".max_position_limit").as_int();
      motor_min_position_limit_map_[id] = this->get_parameter(ns + ".min_position_limit").as_int();
      motor_velocity_i_gain_map_[id] = this->get_parameter(ns + ".velocity_i_gain").as_int();
      motor_velocity_p_gain_map_[id] = this->get_parameter(ns + ".velocity_p_gain").as_int();
      motor_position_d_gain_map_[id] = this->get_parameter(ns + ".position_d_gain").as_int();
      motor_position_i_gain_map_[id] = this->get_parameter(ns + ".position_i_gain").as_int();
      motor_position_p_gain_map_[id] = this->get_parameter(ns + ".position_p_gain").as_int();
      motor_feedforward_2nd_gain_map_[id] = this->get_parameter(ns + ".feedforward_2nd_gain").as_int();
      motor_feedforward_1st_gain_map_[id] = this->get_parameter(ns + ".feedforward_1st_gain").as_int();
    }

    // Create publishers for feedback, separate namespaces for each motor
    for (auto id : motor_ids_) {
      std::string ns = "motor_" + std::to_string(id);
      current_pub_map_[id] = this->create_publisher<std_msgs::msg::Float32>(ns + "/present_current", 10);
      velocity_pub_map_[id] = this->create_publisher<std_msgs::msg::Float32>(ns + "/present_velocity", 10);
      position_pub_map_[id] = this->create_publisher<std_msgs::msg::Float32>(ns + "/present_position", 10);
    }
    
    // Create publisher for joint states
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Subscribers for individual Goal Current commands
    for (auto id : motor_ids_) {
      std::string topic = "motor_" + std::to_string(id) + "/goal_current";
      auto sub = this->create_subscription<std_msgs::msg::Int16>(
        topic,
        10,
        [this, id](const std_msgs::msg::Int16::SharedPtr msg) {
          writeGoalVelocity(id, msg->data);
        }
      );
      goal_current_sub_map_[id] = sub;
    }
    // for (auto id : motor_ids_) {
    //   std::string topic = "motor_" + std::to_string(id) + "/goal_current";
    //   auto sub = this->create_subscription<std_msgs::msg::Int16>(
    //     topic,
    //     10,
    //     [this, id](const std_msgs::msg::Int16::SharedPtr msg) {
    //       uint8_t dxl_error = 0;
    //       int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_GOAL_CURRENT, msg->data, &dxl_error);
    //       if (dxl_comm_result != COMM_SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to write goal current: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    //       } else if (dxl_error != 0) {
    //         RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Goal current write error: %s", id, packetHandler_->getRxPacketError(dxl_error));
    //       } else {
    //         RCLCPP_INFO(this->get_logger(), "Motor ID %d: Set goal current to %d", id, msg->data);
    //       }
    //     }
    //   );
    //   goal_current_sub_map_[id] = sub;
    // }
    
    // Subscriber for Group Goal Current
    group_goal_current_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "group_goal_current", 10,
      std::bind(&MultiMotorControlNode::groupGoalCurrentCallback, this, std::placeholders::_1)
    );


    // Speed Mode subscribers
    for (auto id : motor_ids_) {
      std::string topic = "motor_" + std::to_string(id) + "/goal_velocity";
      auto sub = this->create_subscription<std_msgs::msg::Float32>(
        topic,
        10,
        [this, id](const std_msgs::msg::Float32::SharedPtr msg) {
          writeGoalVelocity(id, msg->data);
        });
      goal_velocity_sub_map_[id] = sub;
    }

    group_goal_velocity_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "group_goal_velocity", 10,
      std::bind(&MultiMotorControlNode::groupGoalVelocityCallback, this, std::placeholders::_1)
    );

    // Position Mode subscribers
    for (auto id : motor_ids_) {
      std::string topic = "motor_" + std::to_string(id) + "/goal_position";
      auto sub = this->create_subscription<std_msgs::msg::Float32>(
        topic,
        10,
        [this, id](const std_msgs::msg::Float32::SharedPtr msg) {
          writeGoalPosition(id, msg->data);
        });
      goal_position_sub_map_[id] = sub;
    }

    group_goal_position_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
      "group_goal_position", 10,
      std::bind(&MultiMotorControlNode::groupGoalPositionCallback, this, std::placeholders::_1)
    );
    // Subscribers for adjusting Velocity PID (array of 2: [I Gain, P Gain])
    // for (auto id : motor_ids_) {
    //   std::string topic = "motor_" + std::to_string(id) + "/set_velocity_pid";
    //   auto sub = this->create_subscription<std_msgs::msg::Int16MultiArray>(
    //     topic,
    //     10,
    //     [this, id](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    //       if (msg->data.size() != 2) {
    //         RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Velocity PID array size is not 2", id);
    //         return;
    //       }
    //       uint8_t dxl_error = 0;
    //       int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_VELOCITY_I_GAIN, msg->data[0], &dxl_error);
    //       if (dxl_comm_result != COMM_SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to set Velocity I Gain: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    //       }
    //       dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_VELOCITY_P_GAIN, msg->data[1], &dxl_error);
    //       if (dxl_comm_result != COMM_SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to set Velocity P Gain: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    //       } else {
    //         RCLCPP_INFO(this->get_logger(), "Motor ID %d: Velocity PID set to [%d, %d]", id, msg->data[0], msg->data[1]);
    //       }
    //     }
    //   );
    //   velocity_pid_sub_map_[id] = sub;
    // }

    // Subscribers for adjusting Position PID (array of 3: [D Gain, I Gain, P Gain])
    // for (auto id : motor_ids_) {
    //   std::string topic = "motor_" + std::to_string(id) + "/set_position_pid";
    //   auto sub = this->create_subscription<std_msgs::msg::Int16MultiArray>(
    //     topic,
    //     10,
    //     [this, id](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    //       if (msg->data.size() != 3) {
    //         RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Position PID array size is not 3", id);
    //         return;
    //       }
    //       uint8_t dxl_error = 0;
    //       int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_POSITION_D_GAIN, msg->data[0], &dxl_error);
    //       if (dxl_comm_result != COMM_SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to set Position D Gain: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    //       }
    //       dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_POSITION_I_GAIN, msg->data[1], &dxl_error);
    //       if (dxl_comm_result != COMM_SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to set Position I Gain: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    //       }
    //       dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_POSITION_P_GAIN, msg->data[2], &dxl_error);
    //       if (dxl_comm_result != COMM_SUCCESS) {
    //         RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to set Position P Gain: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    //       } else {
    //         RCLCPP_INFO(this->get_logger(), "Motor ID %d: Position PID set to [%d, %d, %d]", id, msg->data[0], msg->data[1], msg->data[2]);
    //       }
    //     }
    //   );
    //   position_pid_sub_map_[id] = sub;
    // }

    // Setup Dynamixel Port and Packet Handlers
    std::string DEVICENAME;
    int BAUDRATE;

    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 1000000);

    DEVICENAME = this->get_parameter("port_name").as_string();
    BAUDRATE = this->get_parameter("baudrate").as_int();

    // Configure Dynamixel Port and Baudrate
    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    
    if (!portHandler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "ไม่สามารถเปิดพอร์ตได้");
      return;
    }
    if (!portHandler_->setBaudRate(BAUDRATE)) {
      RCLCPP_ERROR(this->get_logger(), "ไม่สามารถตั้งค่า baudrate ได้");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "เปิดพอร์ตและตั้งค่า baudrate สำเร็จ");

    RCLCPP_INFO(this->get_logger(), "------ Configuring Motor ID: ปิดพอร์ตและตั้งค่า baudrate สำเร็จ");
    // ----- SET MODE for each motor -----
    // Torque should be disabled before setting operating mode
    for (auto id : motor_ids_) {
      uint8_t dxl_error = 0;
      int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 64, 0, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to disable torque for set mode: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
      }
      int op_mode = motor_operating_mode_map_[id];
      dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, op_mode, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to set operating mode: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Operating mode error: %s", id, packetHandler_->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Motor ID %d: Operating mode set to %d", id, op_mode);
      }
      configureMotor(id);
    }
    // After setting operating mode and configuration, torque can be enabled

    // Enable Torque 
    for (auto id : motor_ids_) {
      uint8_t dxl_error = 0;
      int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, 64, 1, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to enable torque: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Torque enable error: %s", id, packetHandler_->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Motor ID %d: Torque enabled", id);
      }
    }
    // ----- END SET MODE -----

    // Create instance of GroupBulkRead for feedback reading from all motors
    groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
    // For each motor, add parameters for Bulk Read (starts at address 126, covering 10 bytes)
    for (auto id : motor_ids_) {
      if (!groupBulkRead_->addParam(id, ADDR_BULK_START, BULK_DATA_LENGTH)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to add parameter for motor ID: %d", id);
      }
    }

    // Create instance of GroupSyncWrite for Goal Current commands as a group
    groupSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_GOAL_CURRENT, 2);

    // Set timer to read feedback (example interval set to 1ms, adjust as needed)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
      std::bind(&MultiMotorControlNode::readFeedback, this));
  }

  void reconnectDynamixel()
  {
    RCLCPP_WARN(this->get_logger(), "Attempting to reconnect Dynamixel...");
    portHandler_->closePort();

    while (rclcpp::ok()) {
      if (portHandler_->openPort() && portHandler_->setBaudRate(BAUDRATE)) {
        RCLCPP_INFO(this->get_logger(), "Dynamixel reconnected successfully.");

        for (auto id : motor_ids_) {
          configureMotor(id);
          uint8_t dxl_error;
          packetHandler_->write1ByteTxRx(portHandler_, id, 64, 1, &dxl_error);
        }

        break;
      }
      RCLCPP_ERROR(this->get_logger(), "Reconnect failed. Retrying in 1 second...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  void writeGoalCurrent(int id, int16_t goal_current)
  {
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_GOAL_CURRENT, goal_current, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Error setting goal current, attempting reconnect... (Result: %s, Error: %s)", 
                  id, packetHandler_->getTxRxResult(dxl_comm_result), packetHandler_->getRxPacketError(dxl_error));
      reconnectDynamixel();
    } else {
      RCLCPP_INFO(this->get_logger(), "Motor ID %d: Goal current set to %d", id, goal_current);
    }
  }

  void writeGoalVelocity(int id, float velocity_rad_s)
  {
    float rev_per_min = (velocity_rad_s * 60.0f) / (2.0f * M_PI);
    int32_t velocity_raw = static_cast<int32_t>(rev_per_min / motor_velocity_factor_map_[id]);
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, 104, velocity_raw, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Error setting velocity, attempting reconnect...", id);
      reconnectDynamixel();
    } else {
      RCLCPP_INFO(this->get_logger(), "Motor ID %d: Goal velocity set to %.2f rad/s", id, velocity_rad_s);
    }
  }

  void writeGoalPosition(int id, float position_rad)
  {
    float position_deg = position_rad * (180.0f / M_PI);
    uint32_t position_raw = static_cast<uint32_t>((position_deg / 360.0f) * motor_position_full_scale_map_[id]);
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, 116, position_raw, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Error setting position, attempting reconnect...", id);
      reconnectDynamixel();
    } else {
      RCLCPP_INFO(this->get_logger(), "Motor ID %d: Goal position set to %.2f rad", id, position_rad);
    }
  }

  void writeGoalVelocity_backup(int id, float velocity_rad_s)
  {
    float rev_per_min = (velocity_rad_s * 60.0f) / (2.0f * M_PI);
    int32_t velocity_raw = static_cast<int32_t>(rev_per_min / motor_velocity_factor_map_[id]);
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, 104, velocity_raw, &dxl_error); // 104: Goal Velocity
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to set goal velocity: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Goal velocity error: %s", id, packetHandler_->getRxPacketError(dxl_error));
    } else {
      RCLCPP_INFO(this->get_logger(), "Motor ID %d: Goal velocity set to %.2f rad/s", id, velocity_rad_s);
    }
  }

  void writeGoalPosition_backup(int id, float position_deg)
  {
    uint32_t position_raw = static_cast<uint32_t>((position_deg / 360.0f) * motor_position_full_scale_map_[id]);
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, 116, position_raw, &dxl_error); // 116: Goal Position
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to set goal position: %s", id, packetHandler_->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Goal position error: %s", id, packetHandler_->getRxPacketError(dxl_error));
    } else {
      RCLCPP_INFO(this->get_logger(), "Motor ID %d: Goal position set to %.2f deg", id, position_deg);
    }
  }

  ~MultiMotorControlNode()
  {
    // Clean up resources
    if (groupBulkRead_ != nullptr) {
      delete groupBulkRead_;
    }
    if (groupSyncWrite_ != nullptr) {
      delete groupSyncWrite_;
    }
    portHandler_->closePort();
  }

private:
  // Function for additional register configuration per motor
  void configureMotor(int id)
  {
    RCLCPP_INFO(this->get_logger(), "------ Configuring Motor ID: %d ------", id);
    struct MotorRegister {
      uint16_t addr;
      uint8_t size;
      std::string name;
      uint32_t target_value;
    };

    std::vector<MotorRegister> configs = {
      {ADDR_OPERATING_MODE, 1, "Operating Mode", motor_operating_mode_map_[id]},
      {ADDR_TEMPERATURE_LIMIT, 1, "Temperature Limit", motor_temperature_limit_map_[id]},
      {ADDR_MAX_VOLTAGE_LIMIT, 2, "Max Voltage Limit", motor_max_voltage_limit_map_[id]},
      {ADDR_MIN_VOLTAGE_LIMIT, 2, "Min Voltage Limit", motor_min_voltage_limit_map_[id]},
      {ADDR_PWM_LIMIT, 2, "PWM Limit", motor_pwm_limit_map_[id]},
      {ADDR_CURRENT_LIMIT, 2, "Current Limit", motor_current_limit_map_[id]},
      {ADDR_VELOCITY_LIMIT, 4, "Velocity Limit", motor_velocity_limit_map_[id]},
      {ADDR_MAX_POSITION_LIMIT, 4, "Max Position Limit", motor_max_position_limit_map_[id]},
      {ADDR_MIN_POSITION_LIMIT, 4, "Min Position Limit", motor_min_position_limit_map_[id]},
      {ADDR_VELOCITY_I_GAIN, 2, "Velocity I Gain", motor_velocity_i_gain_map_[id]},
      {ADDR_VELOCITY_P_GAIN, 2, "Velocity P Gain", motor_velocity_p_gain_map_[id]},
      {ADDR_POSITION_D_GAIN, 2, "Position D Gain", motor_position_d_gain_map_[id]},
      {ADDR_POSITION_I_GAIN, 2, "Position I Gain", motor_position_i_gain_map_[id]},
      {ADDR_POSITION_P_GAIN, 2, "Position P Gain", motor_position_p_gain_map_[id]},
      {ADDR_FEEDFORWARD_2ND_GAIN, 2, "Feedforward 2nd Gain", motor_feedforward_2nd_gain_map_[id]},
      {ADDR_FEEDFORWARD_1ST_GAIN, 2, "Feedforward 1st Gain", motor_feedforward_1st_gain_map_[id]},
    };

    uint8_t dxl_error = 0;
    int dxl_comm_result;

    for (auto config : configs) {
      uint32_t current_value = 0;

      // Read the current value before writing
      if (config.size == 1) {
        uint8_t val;
        dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, config.addr, &val, &dxl_error);
        current_value = val;
      } else if (config.size == 2) {
        uint16_t val;
        dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, config.addr, &val, &dxl_error);
        current_value = val;
      } else if (config.size == 4) {
        uint32_t val;
        dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, config.addr, &val, &dxl_error);
        current_value = val;
      }

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to read %s: %s", id, config.name.c_str(), packetHandler_->getTxRxResult(dxl_comm_result));
        continue;
      } else if (dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Error reading %s: %s", id, config.name.c_str(), packetHandler_->getRxPacketError(dxl_error));
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Motor ID %d: %s current = %u, target = %u", id, config.name.c_str(), current_value, config.target_value);

      // Compare current value with target; skip writing if unchanged
      if (current_value == config.target_value) {
        RCLCPP_INFO(this->get_logger(), "Motor ID %d: %s unchanged, skipping.", id, config.name.c_str());
        continue;
      }

      // Write new value if different
      if (config.size == 1) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, config.addr, config.target_value, &dxl_error);
      } else if (config.size == 2) {
        dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, config.addr, config.target_value, &dxl_error);
      } else if (config.size == 4) {
        dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, config.addr, config.target_value, &dxl_error);
      }

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to set %s: %s", id, config.name.c_str(), packetHandler_->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Error setting %s: %s", id, config.name.c_str(), packetHandler_->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Motor ID %d: %s updated from %u to %u", id, config.name.c_str(), current_value, config.target_value);
      }
    }
    RCLCPP_INFO(this->get_logger(), "------ Finished Configuring Motor ID: %d ------\n", id);
  }

  // Callback for Group Goal Current command
  void groupGoalCurrentCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != motor_ids_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Group goal current array size (%zu) does not match motor count (%zu)",
                   msg->data.size(), motor_ids_.size());
      return;
    }
    groupSyncWrite_->clearParam();
    for (size_t i = 0; i < motor_ids_.size(); i++) {
      int id = motor_ids_[i];
      int16_t goal_current = msg->data[i];
      uint8_t param_goal_current[2];
      param_goal_current[0] = DXL_LOBYTE(goal_current);
      param_goal_current[1] = DXL_HIBYTE(goal_current);
      if (!groupSyncWrite_->addParam(id, param_goal_current)) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Failed to add parameter for group sync write", id);
      }
    }
    int dxl_comm_result = groupSyncWrite_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Group sync write failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
    } else {
      RCLCPP_INFO(this->get_logger(), "Group sync write succeeded");
    }
    groupSyncWrite_->clearParam();
  }

  void groupGoalVelocityCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != motor_ids_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Group goal velocity array size mismatch");
      return;
    }
    dynamixel::GroupSyncWrite groupSyncVelocity(portHandler_, packetHandler_, 104, 4); // 104: Goal Velocity
    for (size_t i = 0; i < motor_ids_.size(); i++) {
      int id = motor_ids_[i];
      int32_t velocity_raw = msg->data[i];
      uint8_t param_velocity[4];
      param_velocity[0] = DXL_LOBYTE(DXL_LOWORD(velocity_raw));
      param_velocity[1] = DXL_HIBYTE(DXL_LOWORD(velocity_raw));
      param_velocity[2] = DXL_LOBYTE(DXL_HIWORD(velocity_raw));
      param_velocity[3] = DXL_HIBYTE(DXL_HIWORD(velocity_raw));
      groupSyncVelocity.addParam(id, param_velocity);
    }
    groupSyncVelocity.txPacket();
  }

  void groupGoalPositionCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != motor_ids_.size()) {
      RCLCPP_ERROR(this->get_logger(), "Group goal position array size mismatch");
      return;
    }
    dynamixel::GroupSyncWrite groupSyncPosition(portHandler_, packetHandler_, 116, 4); // 116: Goal Position
    for (size_t i = 0; i < motor_ids_.size(); i++) {
      int id = motor_ids_[i];
      int32_t position_raw = msg->data[i];
      uint8_t param_position[4];
      param_position[0] = DXL_LOBYTE(DXL_LOWORD(position_raw));
      param_position[1] = DXL_HIBYTE(DXL_LOWORD(position_raw));
      param_position[2] = DXL_LOBYTE(DXL_HIWORD(position_raw));
      param_position[3] = DXL_HIBYTE(DXL_HIWORD(position_raw));
      groupSyncPosition.addParam(id, param_position);
    }
    groupSyncPosition.txPacket();
  }

  // Function to read feedback using GroupBulkRead and publish joint state
  void readFeedback()
  {
    int dxl_comm_result = groupBulkRead_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Bulk read failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
      return;
    }
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->now();
    for (auto id : motor_ids_) {
      if (!groupBulkRead_->isAvailable(id, ADDR_PRESENT_CURRENT, 2)) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Data not available for current", id);
        continue;
      }
      if (!groupBulkRead_->isAvailable(id, ADDR_PRESENT_VELOCITY, 4)) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Data not available for velocity", id);
        continue;
      }
      if (!groupBulkRead_->isAvailable(id, ADDR_PRESENT_POSITION, 4)) {
        RCLCPP_ERROR(this->get_logger(), "Motor ID %d: Data not available for position", id);
        continue;
      }
      int16_t current_raw = groupBulkRead_->getData(id, ADDR_PRESENT_CURRENT, 2);
      float current_mA = static_cast<float>(current_raw) * motor_current_factor_map_[id];
      std_msgs::msg::Float32 current_msg;
      current_msg.data = current_mA;
      current_pub_map_[id]->publish(current_msg);
      int32_t velocity_raw = groupBulkRead_->getData(id, ADDR_PRESENT_VELOCITY, 4);
      float rev_per_min = static_cast<float>(velocity_raw) * motor_velocity_factor_map_[id];
      float velocity_rad_s = (rev_per_min * 2.0f * M_PI) / 60.0f;
      std_msgs::msg::Float32 velocity_msg;
      velocity_msg.data = velocity_rad_s;
      velocity_pub_map_[id]->publish(velocity_msg);
      int32_t position_raw = groupBulkRead_->getData(id, ADDR_PRESENT_POSITION, 4);
      float position_deg = (static_cast<float>(position_raw) * 2.0f * M_PI) / motor_position_full_scale_map_[id];
      std_msgs::msg::Float32 position_msg;
      position_msg.data = position_deg;
      position_pub_map_[id]->publish(position_msg);
      joint_state_msg.name.push_back("motor_" + std::to_string(id));
      joint_state_msg.position.push_back(position_deg);
      joint_state_msg.velocity.push_back(velocity_rad_s);
      joint_state_msg.effort.push_back(current_mA);
    }
    joint_state_pub_->publish(joint_state_msg);
  }

  // Member variables
  std::vector<int> motor_ids_;
  std::map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> current_pub_map_;
  std::map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> velocity_pub_map_;
  std::map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> position_pub_map_;
  
  std::map<int, rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr> goal_current_sub_map_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr group_goal_current_sub_;
  
  std::map<int, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> goal_velocity_sub_map_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr group_goal_velocity_sub_;

  std::map<int, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> goal_position_sub_map_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr group_goal_position_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::map<int, rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr> velocity_pid_sub_map_;
  std::map<int, rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr> position_pid_sub_map_;

  // Per motor configuration maps
  std::map<int, double> motor_current_factor_map_;
  std::map<int, double> motor_velocity_factor_map_;
  std::map<int, double> motor_position_full_scale_map_;
  std::map<int, int>    motor_operating_mode_map_;

  std::map<int, int> motor_temperature_limit_map_;
  std::map<int, int> motor_max_voltage_limit_map_;
  std::map<int, int> motor_min_voltage_limit_map_;
  std::map<int, int> motor_pwm_limit_map_;
  std::map<int, int> motor_current_limit_map_;
  std::map<int, int> motor_velocity_limit_map_;
  std::map<int, int> motor_max_position_limit_map_;
  std::map<int, int> motor_min_position_limit_map_;
  std::map<int, int> motor_velocity_i_gain_map_;
  std::map<int, int> motor_velocity_p_gain_map_;
  std::map<int, int> motor_position_d_gain_map_;
  std::map<int, int> motor_position_i_gain_map_;
  std::map<int, int> motor_position_p_gain_map_;
  std::map<int, int> motor_feedforward_2nd_gain_map_;
  std::map<int, int> motor_feedforward_1st_gain_map_;

  dynamixel::PortHandler* portHandler_;
  dynamixel::PacketHandler* packetHandler_;
  dynamixel::GroupBulkRead* groupBulkRead_;
  dynamixel::GroupSyncWrite* groupSyncWrite_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiMotorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
