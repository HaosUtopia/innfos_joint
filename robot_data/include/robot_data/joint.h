#ifndef ROBOT_DATA_JOINT_H
#define ROBOT_DATA_JOINT_H

#include <hardware_interface/joint_command_interface.h>
#include <urdf/model.h>
#include <ros/ros.h>
#include <robot_utils/robot_utils.h>

namespace robot_data
{

class Joint
{
public:
  Joint() = default;
  ~Joint() = default;
  
  bool init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface);
  bool init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface, ros::NodeHandle nh);\
  bool init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface, bool is_continous, double min_pos, double max_pos, double max_vel, double max_eff);
  bool init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface, bool is_continous, double min_pos, double max_pos, double max_vel, double max_eff, ros::NodeHandle nh);
  void setPositionControlLoop(const bool& loop);
  void setVelocityControlLoop(const bool& loop);
  void setPositionControlParam(const double& Kp, const double& Ki, const double Kd);
  void setVelocityControlParam(const double& Kp, const double& Ki, const double Kd);
  void setErrorThreshold(const double& et); // TODO: setPositionErrorThreshold setVelocityErrorThreshold setEffortErrorThreshold
  void initState();
  void updateState();
  void updateCommand(const double& desired_state, const ros::Duration& period);
  void updateCommand(double* desired_state, const ros::Duration& period);
  void setCommand(const double& command);
  void resetCommand();
  void addCompensationCommand(const double& compensation_command);
  void executeCommand();
  void resetError();
  
  const bool& isContinous() const;
  const double& minPosition() const;
  const double& maxPosition() const;
  const double& maxVelocity() const;
  const double& maxEffort() const;

  const double& currentState() const;
  const double& position() const;
  const double& velocity() const;
  const double& acceleration() const;
  const double& effort() const;
private:
  /// Joint Parameter
  hardware_interface::JointHandle joint_handle_;
  enum JointType {POSITION = 0, VELOCITY = 1, EFFORT = 2, UNDEFINED = 3} joint_type_ = {UNDEFINED};

  /// Joint Limit
  bool is_continous_;
  double min_limit_[3]; // [min_pos, min_vel, min_eff]
  double max_limit_[3]; // [max_pos, max_vel, max_eff]
  
  /// Current Joint State
  double current_state_[4]; // [pos, vel, eff, acc]

  /// Last Joint State
  double last_state_[4]; // [pos, vel, eff, acc (Estimated)]

  /// Reference Joint State
  double reference_state_ = {0.0};
  
  /// Next Joint Command
  double joint_command_ = {0.0};

  /// Control Loop Flag
  bool pos_loop_ = {false};
  bool vel_loop_ = {false};
  bool eff_loop_ = {true};
  
  /// Control Parameter
  double Kpp_ = {0.0};
  double Kpd_ = {0.0};
  double Kpi_ = {0.0};

  double Kvp_ = {0.0};
  double Kvd_ = {0.0};
  double Kvi_ = {0.0};
  
  /// Error For Control
  double epp_ = {0.0};
  double epd_ = {0.0};
  double epi_ = {0.0};

  double evp_ = {0.0};
  double evd_ = {0.0};
  double evi_ = {0.0};

  /// Max Error
  double et_ = {std::numeric_limits<double>::infinity()};
};

}

#endif // ROBOT_DATA_JOINT_H
