#include <innfos_joint_hardware/innfos_joint_hardware.h>

bool JointRobotHardware::init(ros::NodeHandle& nh, ros::NodeHandle& hw_nh)
{
  // Get ros node handle
  nh_ = nh;
  hw_nh_ = hw_nh;

  // Init joint zero position
  hw_nh_.getParam("zero_position", q0_);
  ROS_INFO_STREAM("Set zero position as " << q0_);
  
  // Init the actuators
  ROS_INFO("Initialing the actuator...");
  ActuatorController::initController();
  innfos_sdk_ = ActuatorController::getInstance();

  // Look up all the actutors
  Actuator::ErrorsDefine err_;
  ids_ = innfos_sdk_->lookupActuators(err_);

  if (ids_.empty())
  {
    ROS_ERROR("No actuator found");
    return false;
  }

  // Enable the actuators
  ROS_INFO("Enabling all the actuators...");
  if (!innfos_sdk_->enableAllActuators())
  {
    ROS_ERROR("Failed to enable the actuators");
    return false;
  }

  // Set current mode for all the actuators
  mode_ = Actuator::Mode_Cur;
  innfos_sdk_->activateActuatorModeInBantch(ids_, mode_);

  // Set joint limitation for the actuators
  joint_limits_interface::JointLimits limits;
  joint_limits_interface::SoftJointLimits soft_limits;

  limits.has_position_limits = true;
  limits.min_position = -1.74;
  limits.max_position = 1.74;

  limits.has_velocity_limits = true;
  limits.max_velocity = 1.2;
  
  limits.has_effort_limits = true;
  limits.max_effort = 10.0;

  soft_limits.min_position = -1.6;
  soft_limits.max_position = 1.6;
  soft_limits.k_position = 100;
  soft_limits.k_velocity = 10;

  // Connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle("innfos_joint", &pos_, &vel_, &eff_);
  jnt_state_interface_.registerHandle(state_handle);

  // Connect and register the joint position interface
  hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle("innfos_joint"), &pos_cmd_);
  jnt_pos_interface_.registerHandle(pos_handle);

  // Connect and register the joint position limits interface
  joint_limits_interface::PositionJointSoftLimitsHandle lim_pos_handle(jnt_pos_interface_.getHandle("innfos_joint"), limits, soft_limits);
  lim_pos_interface_.registerHandle(lim_pos_handle);

  // Connect and register the joint velocity interface
  hardware_interface::JointHandle vel_handle(jnt_state_interface_.getHandle("innfos_joint"), &vel_cmd_);
  jnt_vel_interface_.registerHandle(vel_handle);

  // Connect and register the joint velocity limits interface
  joint_limits_interface::VelocityJointSoftLimitsHandle lim_vel_handle(jnt_vel_interface_.getHandle("innfos_joint"), limits, soft_limits);
  lim_vel_interface_.registerHandle(lim_vel_handle);

  // Connect and register the joint effort interface
  hardware_interface::JointHandle eff_handle(jnt_state_interface_.getHandle("innfos_joint"), &eff_cmd_);
  jnt_eff_interface_.registerHandle(eff_handle);

  // Connect and register the joint effort limits interface
  joint_limits_interface::EffortJointSoftLimitsHandle lim_eff_handle(jnt_eff_interface_.getHandle("innfos_joint"), limits, soft_limits);
  lim_eff_interface_.registerHandle(lim_eff_handle);

  // Register interfaces
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_interface_);
  registerInterface(&jnt_vel_interface_);
  registerInterface(&jnt_eff_interface_);
  registerInterface(&lim_pos_interface_);
  registerInterface(&lim_vel_interface_);
  registerInterface(&lim_eff_interface_);

  // Set controller manager
  controller_manager_.reset(new controller_manager::ControllerManager(this, hw_nh_));

  // Set control loop
  control_loop_ = hw_nh_.createTimer(ros::Duration(1.0 / freq_), &JointRobotHardware::update, this);

  return true;
}

JointRobotHardware::~JointRobotHardware()
{
  ROS_INFO("Disabling all the actuators");
  innfos_sdk_->disableAllActuators();
  // delete innfos_sdk_;
}

void JointRobotHardware::update(const ros::TimerEvent& event)
{
  time_ = ros::Time::now();
  period_ = ros::Duration(event.current_real - event.last_real);

  read(time_, period_);
  controller_manager_->update(time_, period_);
  write(time_, period_);
}

void JointRobotHardware::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_controllers, const std::list<hardware_interface::ControllerInfo>& stop_controllers)
{
  for (const auto& controller : start_controllers)
  {
    for (const auto& resource : controller.claimed_resources)
    {
      if (resource.hardware_interface == "hardware_interface::EffortJointInterface")
      {
        mode_ = Actuator::Mode_Cur;;
      }
      else if (resource.hardware_interface == "hardware_interface::VelocityJointInterface")
      {
        mode_ = Actuator::Mode_Vel;
      }
      else if (resource.hardware_interface == "hardware_interface::PositionJointInterface")
      {
        mode_ = Actuator::Mode_Pos;
      }
    }
  }

  innfos_sdk_->activateActuatorModeInBantch(ids_, mode_);
}

void JointRobotHardware::read(const ros::Time& time, const ros::Duration& period)
{
  for (auto id : ids_)
  {
    pos_ = - Kp_ * innfos_sdk_->getPosition(ids_[0].actuatorID, true) + q0_;
    vel_ = - Kv_ * innfos_sdk_->getVelocity(ids_[0].actuatorID, true);
    eff_ = - Kt_ * innfos_sdk_->getCurrent(ids_[0].actuatorID, true);
  }
}

void JointRobotHardware::write(const ros::Time& time, const ros::Duration& period)
{
  switch(mode_)
  {
    case Actuator::Mode_Pos:
    {
      lim_pos_interface_.enforceLimits(period);
      for (auto id : ids_)
      {
        innfos_sdk_->setPosition(id.actuatorID, (- pos_cmd_ - q0_) / Kp_);
      }
      break;
    }
    case Actuator::Mode_Vel:
    {
      lim_vel_interface_.enforceLimits(period);
      for (auto id : ids_)
      {
        innfos_sdk_->setVelocity(id.actuatorID, - vel_cmd_ / Kv_);
      }
      break;
    }
    case Actuator::Mode_Cur:
    {
      //lim_eff_interface_.enforceLimits(period);
      //ROS_INFO_STREAM("current: " << eff_cmd_ / Kt_);
      for (auto id : ids_)
      {
        innfos_sdk_->setCurrent(id.actuatorID, - eff_cmd_ / Kt_);
      }
      break;
    }
    default:
      ROS_WARN("Unsupported mode");
  }
}
