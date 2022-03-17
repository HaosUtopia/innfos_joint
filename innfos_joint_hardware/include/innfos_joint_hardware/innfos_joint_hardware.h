#ifndef INFFOS_JOINT_HARDWARE_INNFOS_JOINT_HARDWARE_H
#define INFFOS_JOINT_HARDWARE_INNFOS_JOINT_HARDWARE_H

// C++ Standard
#include <vector>
#include <memory>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>

// Innfos SDK
#include <actuatorcontroller.h>
#include <actuatordefine.h>

// Hardware Interface
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Joint Limits Interface
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>

// Controller Manager
#include <controller_manager/controller_manager.h>

class JointRobotHardware : public hardware_interface::RobotHW
{
public:
  JointRobotHardware() = default;
  ~JointRobotHardware();

  bool init(ros::NodeHandle& nh, ros::NodeHandle& hw_nh) override;
  void update(const ros::TimerEvent& event);
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_controllers, const std::list<hardware_interface::ControllerInfo>& stop_controllers) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle hw_nh_;
  ros::Timer control_loop_;
  // ros::ServiceServer srv_set_mode_;

  // Innfos SDK
  ActuatorController *innfos_sdk_;

  // Innfos Coefficient
  const double Kp_ = {0.174532925};
  const double Kv_ = {0.002908882};
  const double Kt_ = {2.1816};

  // Joint Zero Position Compensation
  double q0_ = {0.0};

  // Hardware Interface
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::EffortJointInterface jnt_eff_interface_;

  // Joint Limits Interface
  joint_limits_interface::PositionJointSoftLimitsInterface lim_pos_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface lim_vel_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface lim_eff_interface_;

  // Controller Manager
  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;

  // Control Loop
  double freq_ = {100.0};
  ros::Time time_;
  ros::Duration period_;

  // Joint IDs
  std::vector<ActuatorController::UnifiedID> ids_;

  // Joint Mode
  Actuator::ActuatorMode mode_;

  // Joint State
  double pos_;
  double vel_;
  double eff_;

  // Joint Command
  double pos_cmd_;
  double vel_cmd_;
  double eff_cmd_;

private:
  // TODO: Add mode change callback
  // bool setModeCallback()
};

#endif // INFFOS_JOINT_HARDWARE_INNFOS_JOINT_HARDWARE_H