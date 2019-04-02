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

#include "pacmod_interface.h"

PacmodInterface::PacmodInterface()
  : nh_()
  , private_nh_("~")
  , engage_cmd_(false)
  , engage_state_(false)
  , prev_engage_state_(false)
  , enable_(false)
  , ignore_overrides_(false)
  , clear_override_(false)
  , clear_faults_(false)
  , init_vehicle_cmd_(false)
{
  private_nh_.param<bool>("debug", debug_, false);
  private_nh_.param<double>("loop_rate", loop_rate_, 100.0);
  private_nh_.param<double>("accel_kp", accel_kp_, 0.2);
  private_nh_.param<double>("accel_ki", accel_ki_, 0.1);
  private_nh_.param<double>("accel_kd", accel_kd_, 0.0);
  private_nh_.param<double>("accel_max", accel_max_, 0.3);
  private_nh_.param<double>("brake_kp", brake_kp_, 0.1);
  private_nh_.param<double>("brake_ki", brake_ki_, 0.1);
  private_nh_.param<double>("brake_kd", brake_kd_, 0.0);
  private_nh_.param<double>("brake_max", brake_max_, 0.5);
  private_nh_.param<double>("brake_deadband", brake_deadband_, 1.39);
  private_nh_.param<double>("rotation_rate", rotation_rate_, 6.0);
  private_nh_.param<std::string>("speed_control_mode", speed_control_mode_str_, "table");
  private_nh_.param<std::string>("throttle_table_path", throttle_table_path_, "throttle.csv");
  private_nh_.param<std::string>("brake_table_path", brake_table_path_, "brake.csv");

  if (speed_control_mode_str_ == "pid")
  {
    speed_control_mode_ = SpeedControlMode::PID;

    accel_pid_.setGain(accel_kp_, accel_ki_, accel_kd_);
    brake_pid_.setGain(brake_kp_, brake_ki_, brake_kd_);
    accel_pid_.setMinMax(0.0, accel_max_);
    brake_pid_.setMinMax(0.0, brake_max_);
  }
  else if (speed_control_mode_str_ == "table")
  {
    speed_control_mode_ = SpeedControlMode::TABLE;
    acceleration_pid_.setGain(0.1, 0.0, 0.0);
    acceleration_pid_.setMinMax(0.0, 3.0);
    if (table_controller_.loadTables(throttle_table_path_, brake_table_path_))
    {
      ROS_ERROR("Loading tables is failed.");
      ros::shutdown();
    }
    std::cout << table_controller_.getThrottle(1.0, 0.3) << std::endl;
    std::cout << table_controller_.getBrake(2.0, -0.5) << std::endl;
  }
  else
  {
    ROS_ERROR("'%s' is not supported speed control mode.", speed_control_mode_str_.c_str());
    ros::shutdown();
  }

  rate_ = new ros::Rate(loop_rate_);

  // from autoware
  vehicle_cmd_sub_ = nh_.subscribe("vehicle_cmd", 1, &PacmodInterface::callbackVehicleCmd, this);
  engage_cmd_sub_ = nh_.subscribe("vehicle/engage", 1, &PacmodInterface::callbackEngage, this);

  // from pacmod
  pacmod_enabled_sub_ = nh_.subscribe("pacmod/as_tx/enabled", 1, &PacmodInterface::callbackPacmodEnabled, this);
  pacmod_wheel_sub_ =
      new message_filters::Subscriber<pacmod_msgs::WheelSpeedRpt>(nh_, "pacmod/parsed_tx/wheel_speed_rpt", 1);
  pacmod_steer_sub_ =
      new message_filters::Subscriber<pacmod_msgs::SystemRptFloat>(nh_, "pacmod/parsed_tx/steer_rpt", 1);

  pacmod_twist_sync_ = new message_filters::Synchronizer<PacmodTwistSyncPolicy>(PacmodTwistSyncPolicy(10),
                                                                                *pacmod_wheel_sub_, *pacmod_steer_sub_);
  pacmod_twist_sync_->registerCallback(boost::bind(&PacmodInterface::callbackPacmodTwist, this, _1, _2));

  // to autoware
  vehicle_status_pub_ = nh_.advertise<autoware_msgs::VehicleStatus>("vehicle_status", 10);
  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("as_current_twist", 10);

  // to pacmod
  pacmod_steer_pub_ = nh_.advertise<pacmod_msgs::SteerSystemCmd>("pacmod/as_rx/steer_cmd", 10);
  pacmod_accel_pub_ = nh_.advertise<pacmod_msgs::SystemCmdFloat>("pacmod/as_rx/accel_cmd", 10);
  pacmod_brake_pub_ = nh_.advertise<pacmod_msgs::SystemCmdFloat>("pacmod/as_rx/brake_cmd", 10);
  pacmod_shift_pub_ = nh_.advertise<pacmod_msgs::SystemCmdInt>("pacmod/as_rx/shift_cmd", 10);
  pacmod_turn_pub_ = nh_.advertise<pacmod_msgs::SystemCmdInt>("pacmod/as_rx/turn_cmd", 10);
  // pacmod_headlight_pub_ = nh_.advertise<pacmod_msgs::SystemCmdInt>("pacmod/as_rx/headlight_cmd", 10);
  // pacmod_horn_pub_ = nh_.advertise<pacmod_msgs::SystemCmdBool>("pacmod/as_rx/horn_cmd", 10);
  // pacmod_wiper_pub_ = nh_.advertise<pacmod_msgs::SystemCmdInt>("pacmod/as_rx/wiper_cmd", 10);

  if (debug_)
  {
    debug_accel_pub_ = private_nh_.advertise<std_msgs::Float32MultiArray>("debug/accel", 10);
    debug_brake_pub_ = private_nh_.advertise<std_msgs::Float32MultiArray>("debug/brake", 10);
  }
}

PacmodInterface::~PacmodInterface()
{
}

void PacmodInterface::run()
{
  while (ros::ok())
  {
    ros::spinOnce();

    if (!checkInitialized())
    {
      ROS_ERROR("Not initialized, waiting for topics...");
      ros::Duration(1.0).sleep();
    }

    updateEngage();
    publishPacmodSteer(vehicle_cmd_);
    publishPacmodAccel(vehicle_cmd_);
    publishPacmodBrake(vehicle_cmd_);
    publishPacmodShift(vehicle_cmd_);
    publishPacmodTurn(vehicle_cmd_);

    rate_->sleep();
  }
}

bool PacmodInterface::checkInitialized()
{
  return (init_vehicle_cmd_);
}

void PacmodInterface::updateEngage()
{
  enable_ = engage_cmd_;
  ignore_overrides_ = false;

  if (engage_cmd_ && !prev_engage_cmd_)
  {
    clear_override_ = true;
    clear_faults_ = true;
  }
  else
  {
    clear_override_ = false;
    clear_faults_ = false;
  }

  prev_engage_cmd_ = engage_cmd_;
}

void PacmodInterface::publishPacmodSteer(const autoware_msgs::VehicleCmd& msg)
{
  static pacmod_msgs::SteerSystemCmd steer;

  steer.header = msg.header;
  steer.header.stamp = ros::Time::now();
  steer.enable = enable_;
  steer.ignore_overrides = ignore_overrides_;
  steer.clear_override = clear_override_;
  steer.clear_faults = clear_faults_;

  steer.command = msg.ctrl_cmd.steering_angle * STEERING_GEAR_RATIO;
  // TODO, default max = 3.3, 4.71239 is fast but jerky
  steer.rotation_rate = rotation_rate_;

  pacmod_steer_pub_.publish(steer);

  ROS_INFO("STEER: target = %f, actual = %f, command = %f", msg.ctrl_cmd.steering_angle, current_steer_, steer.command);
}

void PacmodInterface::publishPacmodAccel(const autoware_msgs::VehicleCmd& msg)
{
  static pacmod_msgs::SystemCmdFloat accel;
  static double error;

  accel.header = msg.header;
  accel.header.stamp = ros::Time::now();
  accel.enable = enable_;
  accel.ignore_overrides = ignore_overrides_;
  accel.clear_override = clear_override_;
  accel.clear_faults = clear_faults_;

  if (speed_control_mode_ == SpeedControlMode::PID)
  {
    error = msg.ctrl_cmd.linear_velocity - current_speed_;
    if (error >= 0)
    {
      accel.command = accel_pid_.update(error, 1.0 / loop_rate_);
    }
    else
    {
      accel_pid_.reset();
      accel.command = 0.0;
    }
  }
  else if (speed_control_mode_ == SpeedControlMode::TABLE)
  {
  }

  pacmod_accel_pub_.publish(accel);

  ROS_INFO("ACCEL: target = %f, actual = %f, error = %f, command = %f", msg.ctrl_cmd.linear_velocity, current_speed_,
           error, accel.command);

  if (debug_)
  {
    std_msgs::Float32MultiArray debug;
    debug.data.push_back(msg.ctrl_cmd.linear_velocity);  // target [m/s]
    debug.data.push_back(current_speed_);                // actual [m/s]
    debug.data.push_back(error);                         // error [m/s]
    debug.data.push_back(accel.command);                 // command [-]
    debug_accel_pub_.publish(debug);
  }
}

void PacmodInterface::publishPacmodBrake(const autoware_msgs::VehicleCmd& msg)
{
  static pacmod_msgs::SystemCmdFloat brake;
  static double error;

  brake.header = msg.header;
  brake.header.stamp = ros::Time::now();
  brake.enable = enable_;
  brake.ignore_overrides = ignore_overrides_;
  brake.clear_override = clear_override_;
  brake.clear_faults = clear_faults_;

  if (speed_control_mode_ == SpeedControlMode::PID)
  {
    error = msg.ctrl_cmd.linear_velocity - current_speed_;
    if (error < -brake_deadband_)
    {
      brake.command = brake_pid_.update(-error, 1.0 / loop_rate_);
    }
    else
    {
      brake_pid_.reset();
      brake.command = 0.0;
    }
  }
  else if (speed_control_mode_ == SpeedControlMode::TABLE)
  {
  }

  pacmod_brake_pub_.publish(brake);

  ROS_INFO("BRAKE: target = %f, actual = %f, error = %f, command = %f", msg.ctrl_cmd.linear_velocity, current_speed_,
           error, brake.command);

  if (debug_)
  {
    std_msgs::Float32MultiArray debug;
    debug.data.push_back(msg.ctrl_cmd.linear_velocity);  // target [m/s]
    debug.data.push_back(current_speed_);                // actual [m/s]
    debug.data.push_back(error);                         // error [m/s]
    debug.data.push_back(brake.command);                 // command [-]
    debug_brake_pub_.publish(debug);
  }
}

void PacmodInterface::publishPacmodShift(const autoware_msgs::VehicleCmd& msg)
{
  static pacmod_msgs::SystemCmdInt shift;

  shift.header = msg.header;
  shift.header.stamp = ros::Time::now();
  shift.enable = enable_;
  shift.ignore_overrides = ignore_overrides_;
  shift.clear_override = clear_override_;
  shift.clear_faults = clear_faults_;

  if (msg.gear == 0)
  {
    shift.command = pacmod_msgs::SystemCmdInt::SHIFT_PARK;
  }
  else if (msg.gear == 1)
  {
    shift.command = pacmod_msgs::SystemCmdInt::SHIFT_FORWARD;
  }
  else if (msg.gear == 2)
  {
    shift.command = pacmod_msgs::SystemCmdInt::SHIFT_REVERSE;
  }
  else if (msg.gear == 4)
  {
    shift.command = pacmod_msgs::SystemCmdInt::SHIFT_NEUTRAL;
  }

  pacmod_shift_pub_.publish(shift);
}

void PacmodInterface::publishPacmodTurn(const autoware_msgs::VehicleCmd& msg)
{
  static pacmod_msgs::SystemCmdInt turn;

  turn.header = msg.header;
  turn.header.stamp = ros::Time::now();
  turn.enable = enable_;
  turn.ignore_overrides = ignore_overrides_;
  turn.clear_override = clear_override_;
  turn.clear_faults = clear_faults_;

  if (msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 0)
  {
    turn.command = pacmod_msgs::SystemCmdInt::TURN_NONE;
  }
  else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 0)
  {
    turn.command = pacmod_msgs::SystemCmdInt::TURN_LEFT;
  }
  else if (msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 1)
  {
    turn.command = pacmod_msgs::SystemCmdInt::TURN_RIGHT;
  }
  else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 1)
  {
    turn.command = pacmod_msgs::SystemCmdInt::TURN_HAZARDS;
  }

  pacmod_turn_pub_.publish(turn);
}

void PacmodInterface::callbackVehicleCmd(const autoware_msgs::VehicleCmd::ConstPtr& msg)
{
  if (!init_vehicle_cmd_)
  {
    init_vehicle_cmd_ = true;
  }

  vehicle_cmd_ = *msg;
}

void PacmodInterface::callbackEngage(const std_msgs::Bool::ConstPtr& msg)
{
  engage_cmd_ = msg->data;
}

void PacmodInterface::callbackPacmodEnabled(const std_msgs::Bool::ConstPtr& msg)
{
  engage_state_ = msg->data;

  if (!engage_state_ && prev_engage_state_)
  {
    engage_cmd_ = false;
  }

  prev_engage_state_ = engage_state_;
}

void PacmodInterface::callbackPacmodTwist(const pacmod_msgs::WheelSpeedRpt::ConstPtr& wheel,
                                          const pacmod_msgs::SystemRptFloat::ConstPtr& steer)
{
  static double lv, az;

  current_speed_ = TIRE_RADIUS * (wheel->rear_left_wheel_speed + wheel->rear_right_wheel_speed) / 2.0;
  current_steer_ = steer->output / STEERING_GEAR_RATIO;

  lv = current_speed_;
  az = std::tan(current_steer_) * current_speed_ / WHEEL_BASE;

  current_twist_.header.stamp = steer->header.stamp;
  current_twist_.header.frame_id = "base_link";
  current_twist_.twist.linear.x = lv;
  current_twist_.twist.angular.z = az;

  current_twist_pub_.publish(current_twist_);
}
