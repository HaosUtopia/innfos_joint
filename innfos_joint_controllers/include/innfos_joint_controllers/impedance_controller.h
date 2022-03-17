#ifndef INNFOS_JOINT_CONTROLLERS_IMPEDANCE_CONTROLLER_H
#define INNFOS_JOINT_CONTROLLERS_IMPEDANCE_CONTROLLER_H

// C++ standard
#include <cmath>

// ROS control
#include <controller_interface/controller.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>

// ROS messages
#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_msgs/SetCommand.h>
#include <std_msgs/Float64.h>

// Dynamic Reconfiguration
#include <dynamic_reconfigure/server.h>
#include <innfos_joint_controllers/ImpedanceControllerParamConfig.h>

// Joint controller
#include <robot_data/joint.h>

namespace innfos_joint_controllers
{

class ImpedanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  ImpedanceController() = default;
  ~ImpedanceController() = default;

  bool init(hardware_interface::EffortJointInterface* eff_joint_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);

  std::string getEffortInterfaceType() const;

private:
  /// Joint
  robot_data::Joint joint_;

  /// Joint Name
  std::string joint_name_ = {"innfos_joint"};

  /// Joint Limits
  double min_pos_;
  double max_pos_;
  double max_vel_;
  double max_eff_;

  /// Desired State
  double desired_state_[3]; // [pos, vel, acc]
  double& q1d = {desired_state_[0]};
  double& qp1d = {desired_state_[1]};
  double& qpp1d = {desired_state_[2]};

  // Actual State
  const double& q1 = {joint_.position()};
  const double& qp1 = {joint_.velocity()};
  const double& qpp1 = {joint_.acceleration()};
  const double& qt1 = {joint_.effort()};

  /// Desired Effort
  double desired_tau_ = {0.0};
  double& qt1d = desired_tau_;

  /// External Effort
  double external_tau_ = {0.0};
  double& qt1ext = external_tau_;

  /// Impedance parameters (Desired)
  double Md = {0.0};
  double Dd = {0.0};
  double Kd = {0.0};

  /// Robot Parameters
  double g = {9.794};
  double gx = {0.0};
  double gy = {0.0};
  double gz = {-1.0};

  double m1 = {0.56829};

  double L1 = {0.5154};
  double L2 = {0.2577};

  double I111 = {0.27142};
  double I112 = {0.0};
  double I113 = {0.0};
  double I122 = {0.27157};
  double I123 = {0.0};
  double I133 = {0.023627};

  /// Dynamic Parameters
  double M;
  double C;
  double G;

  /// Dynamic Reconfiguration
  boost::shared_ptr<dynamic_reconfigure::Server<innfos_joint_controllers::ImpedanceControllerParamConfig> > dynamic_server_;
  dynamic_reconfigure::Server<innfos_joint_controllers::ImpedanceControllerParamConfig>::CallbackType dynamic_callback_;

  /// Publishers and Subscribers
  ros::Publisher pub_state_;
  ros::Publisher pub_torque_;

  /// Service
  ros::ServiceServer srv_command_;

  /// State Publish Time
  ros::Time last_state_published_time_;
  ros::Duration state_publish_period_;
  
  /// Messages
  control_msgs::JointTrajectoryControllerState state_;
  std_msgs::Float64 tau_ext_;

  void evaluateInertia();
  void evaluateCoriolisCentripetal();
  void evaluateGravity();
  void evaluateExternalTorque();
  void evaluateDynamics();

  void updateState();
  void calculateTorque();
  void executeTorque();
  
  void publishState(const ros::Time& time);

  void setConfigCallback(innfos_joint_controllers::ImpedanceControllerParamConfig &config, uint32_t level);
  bool setCommandCallback(controller_msgs::SetCommand::Request& req,
                          controller_msgs::SetCommand::Response& resp);
};

PLUGINLIB_EXPORT_CLASS(innfos_joint_controllers::ImpedanceController, controller_interface::ControllerBase);

}

#endif // INNFOS_JOINT_CONTROLLERS_IMPEDANCE_CONTROLLER_H