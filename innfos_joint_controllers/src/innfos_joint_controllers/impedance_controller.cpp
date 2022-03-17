#include <innfos_joint_controllers/impedance_controller.h>

namespace innfos_joint_controllers
{

bool ImpedanceController::init(hardware_interface::EffortJointInterface* eff_joint_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // get joint limits
  if (!controller_nh.getParam("min_position", min_pos_))
  {
    ROS_ERROR_STREAM("Could not find 'min_position' parameter (namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("max_position", max_pos_))
  {
    ROS_ERROR_STREAM("Could not find 'max_position' parameter (namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("max_velocity", max_vel_))
  {
    ROS_ERROR_STREAM("Could not find 'max_velocity' parameter (namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("max_effort", max_eff_))
  {
    ROS_ERROR_STREAM("Could not find 'max_effort' parameter (namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  // init joint
  if (!joint_.init(joint_name_, "effort", eff_joint_hw, false, min_pos_, max_pos_, max_vel_, max_eff_))
  {
    ROS_ERROR_STREAM("Initializing joint " << joint_name_ << " failed");
    return false;
  }

  // get control parameters
  if (!controller_nh.getParam("imp_param/M", Md))
  {
    ROS_ERROR_STREAM("Could not find 'imp_param/M' parameter(namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("imp_param/D", Dd))
  {
    ROS_ERROR_STREAM("Could not find 'imp_param/D' parameter(namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("imp_param/K", Kd))
  {
    ROS_ERROR_STREAM("Could not find 'imp_param/K' parameter(namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  // set dynamic reconfiguration server
  dynamic_server_.reset(new dynamic_reconfigure::Server<innfos_joint_controllers::ImpedanceControllerParamConfig>(controller_nh));
  dynamic_callback_ = boost::bind(&ImpedanceController::setConfigCallback, this, _1, _2);
  dynamic_server_->setCallback(dynamic_callback_);

  // get service server name
  std::string command_topic = "set_position";
  if (!controller_nh.getParam("command_topic", command_topic))
  {
    ROS_WARN_STREAM("Could not find 'command_topic' parameter (namespace: " << controller_nh.getNamespace() << "), \"" << command_topic << "\" will be set as default");
  }

  // Get state publish rate
  double state_publish_rate = 50.0;
  if (!controller_nh.getParam("state_publish_rate", state_publish_rate))
  {
    ROS_WARN_STREAM("Could not find 'state_publish_rate' parameter(namespace: " << controller_nh.getNamespace() << "), " << state_publish_rate << " will be set as default");
  }
  state_publish_period_ = ros::Duration(1.0 / state_publish_rate);

  // Initialize state message
  state_.joint_names = {joint_name_};
  state_.desired.positions.resize(1);
  state_.desired.velocities.resize(1);
  state_.desired.accelerations.resize(1);
  state_.desired.effort.resize(1);
  state_.actual.positions.resize(1);
  state_.actual.velocities.resize(1);
  state_.actual.accelerations.resize(1);
  state_.actual.effort.resize(1);
  state_.error.positions.resize(1);
  state_.error.velocities.resize(1);
  state_.error.accelerations.resize(1);
  state_.error.effort.resize(1);

  // Start the publishers and subscribers
  pub_state_ = controller_nh.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
  pub_torque_ = controller_nh.advertise<std_msgs::Float64>("tau_ext", 1);

  // Start the service server
  srv_command_ = controller_nh.advertiseService(command_topic, &ImpedanceController::setCommandCallback, this);

  return true;
}

void ImpedanceController::starting(const ros::Time& time)
{
  last_state_published_time_ = time;

  joint_.initState();
  joint_.resetError();
  q1d = 0.0;
  qp1d = 0.0;
  qpp1d = 0.0;
}

void ImpedanceController::update(const ros::Time& time, const ros::Duration& period)
{
  // if ( (time.nsec % period.nsec) == 0)
  {
    updateState();
    calculateTorque();
    executeTorque();

    publishState(time);
  }
}

void ImpedanceController::stopping(const ros::Time& time)
{

}

void ImpedanceController::evaluateInertia()
{
  M = I133 + L2 * L2 * m1;
}

void ImpedanceController::evaluateCoriolisCentripetal()
{
  C = 0;
}

void ImpedanceController::evaluateGravity()
{
  G = L2 * g * m1 * (gy * cos(q1) + gz * sin(q1));
}

void ImpedanceController::evaluateExternalTorque()
{
  qt1ext = qt1 - M * qpp1 - C * qp1 - G;

  tau_ext_.data = qt1ext;
  pub_torque_.publish(tau_ext_);

  if (qt1ext > 2.0)
  {
    ROS_INFO_STREAM("External force is: " << qt1ext);
    ROS_INFO_STREAM("qt1 is: " << qt1);
    ROS_INFO_STREAM("M is: " << M);
    ROS_INFO_STREAM("qpp1 is: " << qpp1);
    ROS_INFO_STREAM("G is: " << G);
  }
  else
  {
    qt1ext = 0.0;
  }
}

void ImpedanceController::evaluateDynamics()
{
  evaluateInertia();
  evaluateCoriolisCentripetal();
  evaluateGravity();
  evaluateExternalTorque();
}

void ImpedanceController::updateState()
{
  joint_.updateState();
  evaluateDynamics();
}

void ImpedanceController::calculateTorque()
{
  // qt1d = M * qpp1d + C * qp1 + G + M / Md * (Dd * (qp1d - qp1) + Kd * (q1d - q1)) + (1 - M / Md) * qt1ext;
  qt1d = M * qpp1d + C * qp1 + G + M / Md * (Dd * (qp1d - qp1) + Kd * (q1d - q1));
}

void ImpedanceController::executeTorque()
{
  joint_.setCommand(desired_tau_);
  joint_.executeCommand();
}

void ImpedanceController::publishState(const ros::Time& time)
{
  if (last_state_published_time_ + state_publish_period_ < time)
  {
    last_state_published_time_ += state_publish_period_;
    
    state_.header.stamp = time;
    
    state_.desired.positions[0] = q1d;
    state_.desired.velocities[0] = qp1d;
    state_.desired.accelerations[0] = qpp1d;
    state_.desired.effort[0] = qt1d;

    state_.actual.positions[0] = q1;
    state_.actual.velocities[0] = qp1;
    state_.actual.accelerations[0] = qpp1;
    state_.actual.effort[0] = qt1;

    state_.error.positions[0] = state_.desired.positions[0] - state_.actual.positions[0];
    state_.error.velocities[0] = state_.desired.velocities[0] - state_.actual.velocities[0];
    state_.error.accelerations[0] = state_.desired.accelerations[0] - state_.actual.accelerations[0];
    state_.error.effort[0] = state_.desired.effort[0] - state_.actual.effort[0];
    
    pub_state_.publish(state_);
  }
}

void ImpedanceController::setConfigCallback(innfos_joint_controllers::ImpedanceControllerParamConfig &config, uint32_t level)
{
  if (level == 0)
  {
    Md = config.M;
    Dd = config.D;
    Kd = config.K;
    ROS_INFO("Set impedance parameter successfully");
  }
}

bool ImpedanceController::setCommandCallback(controller_msgs::SetCommand::Request& req,
                                             controller_msgs::SetCommand::Response& resp)
{
  if (req.command.command.values.size() != 1)
  {
    ROS_ERROR_STREAM("The command size " << req.command.command.values.size() << " is not equal to joint number " << 1);
    resp.success = false;
    resp.status_message = "Command size doesn't match";
    return true;
  }
  
  q1d = req.command.command.values[0];

  resp.success = true;
  return true;
}

}