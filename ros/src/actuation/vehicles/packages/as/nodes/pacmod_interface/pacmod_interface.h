/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PACMOD_INTERFACE_H
#define PACMOD_INTERFACE_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>

#include <pacmod_msgs/SystemCmdBool.h>
#include <pacmod_msgs/SystemCmdInt.h>
#include <pacmod_msgs/SystemCmdFloat.h>
#include <pacmod_msgs/SteerSystemCmd.h>
#include <pacmod_msgs/SystemRptInt.h>
#include <pacmod_msgs/SystemRptFloat.h>
#include <pacmod_msgs/WheelSpeedRpt.h>

#include "pid_controller.h"
#include "table_controller.h"

// Lexus 450hL parameters
const static double WHEEL_BASE = 2.79;               // [m]
const static double MINIMUM_TURNING_RADIUS = 5.9;    // [m]
const static double MAX_STEERING_WHEEL_ANGLE = 8.6;  // [rad]
const static double STEERING_GEAR_RATIO              // [-]
  = MAX_STEERING_WHEEL_ANGLE / std::asin(WHEEL_BASE / MINIMUM_TURNING_RADIUS);
const static double TIRE_RADIUS = 0.77 / 2.0;        // [m]

typedef enum SPEED_CONTROL_MODE
{
  PID = 0,
  TABLE = 1,
} SpeedControlMode;

class PacmodInterface
{
public:
  PacmodInterface();
  ~PacmodInterface();

  void run();

private:
  // typedefs
  typedef message_filters::sync_policies::ApproximateTime<pacmod_msgs::WheelSpeedRpt, pacmod_msgs::SystemRptFloat>
      PacmodTwistSyncPolicy;

  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // subscribers from autoware
  ros::Subscriber vehicle_cmd_sub_;
  ros::Subscriber engage_cmd_sub_;

  // subscribers from pacmod
  ros::Subscriber pacmod_enabled_sub_;
  message_filters::Subscriber<pacmod_msgs::WheelSpeedRpt>* pacmod_wheel_sub_;
  message_filters::Subscriber<pacmod_msgs::SystemRptFloat>* pacmod_steer_sub_;
  message_filters::Synchronizer<PacmodTwistSyncPolicy>* pacmod_twist_sync_;

  // publishers to autoware
  ros::Publisher vehicle_status_pub_;
  ros::Publisher current_twist_pub_;

  // publishers to pacmod
  ros::Publisher pacmod_steer_pub_;
  ros::Publisher pacmod_accel_pub_;
  ros::Publisher pacmod_brake_pub_;
  ros::Publisher pacmod_shift_pub_;
  ros::Publisher pacmod_turn_pub_;
  // ros::Publisher pacmod_headlight_pub_;
  // ros::Publisher pacmod_horn_pub_;
  // ros::Publisher pacmod_wiper_pub_;

  // publishers for debug
  ros::Publisher debug_accel_pub_;
  ros::Publisher debug_brake_pub_;

  // rosparams
  bool debug_;
  double loop_rate_;
  double rotation_rate_;
  double pid_throttle_kp_, pid_throttle_ki_, pid_throttle_kd_;
  double pid_brake_kp_, pid_brake_ki_, pid_brake_kd_;
  double pid_throttle_max_, pid_brake_max_;
  double pid_brake_deadband_;
  double table_accleration_kp_, table_accleration_ki_, table_accleration_kd_;
  double table_speed_min_, table_speed_max_, table_accleration_min_, table_accleration_max_;
  double table_throttle_max_, table_brake_max_;
  std::string speed_control_mode_str_;
  std::string throttle_table_path_;
  std::string brake_table_path_;

  // variables
  SpeedControlMode speed_control_mode_;
  bool engage_cmd_, prev_engage_cmd_;
  bool engage_state_, prev_engage_state_;

  bool enable_;
  bool ignore_overrides_;
  bool clear_override_;
  bool clear_faults_;

  bool init_vehicle_cmd_;
  double current_speed_, current_steer_;
  double desired_acceleration_;
  geometry_msgs::TwistStamped current_twist_;
  autoware_msgs::VehicleCmd vehicle_cmd_;

  pacmod_msgs::SteerSystemCmd pacmod_steer_;
  pacmod_msgs::SystemCmdFloat pacmod_accel_;
  pacmod_msgs::SystemCmdFloat pacmod_brake_;
  pacmod_msgs::SystemCmdInt pacmod_shift_;
  pacmod_msgs::SystemCmdInt pacmod_turn_;

  ros::Rate* rate_;

  PIDController throttle_pid_;
  PIDController brake_pid_;
  PIDController acceleration_pid_;
  TableController table_controller_;

  // callbacks from autoware
  void callbackVehicleCmd(const autoware_msgs::VehicleCmd::ConstPtr& msg);
  void callbackEngage(const std_msgs::Bool::ConstPtr& msg);

  // callbacks from pacmod
  void callbackPacmodEnabled(const std_msgs::Bool::ConstPtr& msg);
  void callbackPacmodTwist(const pacmod_msgs::WheelSpeedRpt::ConstPtr& wheel,
                           const pacmod_msgs::SystemRptFloat::ConstPtr& steer);

  // functions
  bool checkInitialized();
  void updateEngage();
  void publishPacmodSteer(const autoware_msgs::VehicleCmd& msg);
  void publishPacmodAccel(const autoware_msgs::VehicleCmd& msg);
  void publishPacmodBrake(const autoware_msgs::VehicleCmd& msg);
  void publishPacmodShift(const autoware_msgs::VehicleCmd& msg);
  void publishPacmodTurn(const autoware_msgs::VehicleCmd& msg);
};

#endif  // PACMOD_INTERFACE_H
