#include <innfos_joint_controllers/effort_state_controller.h>

namespace innfos_joint_controllers
{

bool EffortStateController::init(hardware_interface::EffortJointInterface* eff_joint_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // get state type
  if (!controller_nh.getParam("state_type", state_type_))
  {
    ROS_ERROR_STREAM("Could not find 'state_type' parameter (namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (state_type_ == "position")
  {
    state_mode_ = 0;
  }
  else if (state_type_ == "velocity")
  {
    state_mode_ = 1;
  }
  else if (state_type_ == "effort")
  {
    state_mode_ = 2;
  }
  else
  {
    ROS_ERROR("Invalid 'state_type' parameter");
    return false;
  }

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
  if (!joint_.init(joint_name_, state_type_, eff_joint_hw, false, min_pos_, max_pos_, max_vel_, max_eff_))
  {
    ROS_ERROR_STREAM("Initializing joint " << joint_name_ << " failed");
    return false;
  }

  // get control parameters
  if (!controller_nh.getParam("pos_pid/p", Kpp_))
  {
    ROS_ERROR_STREAM("Could not find 'pid/p' parameter(namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("pos_pid/d", Kpd_))
  {
    ROS_ERROR_STREAM("Could not find 'pid/d' parameter(namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("pos_pid/i", Kpi_))
  {
    ROS_ERROR_STREAM("Could not find 'pid/i' parameter(namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("vel_pid/p", Kvp_))
  {
    ROS_ERROR_STREAM("Could not find 'pid/p' parameter(namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("vel_pid/d", Kvd_))
  {
    ROS_ERROR_STREAM("Could not find 'pid/d' parameter(namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  if (!controller_nh.getParam("vel_pid/i", Kvi_))
  {
    ROS_ERROR_STREAM("Could not find 'pid/i' parameter(namespace: " << controller_nh.getNamespace() << ")");
    return false;
  }

  // set pid parameters for joint
  if (Kpp_ || Kpi_ || Kpd_)
  {
    joint_.setPositionControlParam(Kpp_, Kpi_, Kpd_);
    joint_.setPositionControlLoop(true);
  }

  if (Kvp_ || Kvi_ || Kvd_)
  {
    joint_.setVelocityControlParam(Kvp_, Kvi_, Kvd_);
    joint_.setVelocityControlLoop(true);
  }

  // set error threshold for joint
  joint_.setErrorThreshold(0.02);

  // set dynamic reconfiguration server
  dynamic_server_.reset(new dynamic_reconfigure::Server<innfos_joint_controllers::EffortStateControllerPIDConfig>(controller_nh));
  dynamic_callback_ = boost::bind(&EffortStateController::setConfigCallback, this, _1, _2);
  dynamic_server_->setCallback(dynamic_callback_);

  // get service server name
  std::string command_topic = "set_command";
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

  // Start the service server
  srv_command_ = controller_nh.advertiseService(command_topic, &EffortStateController::setCommandCallback, this);

  return true;
}

void EffortStateController::starting(const ros::Time& time)
{
  last_state_published_time_ = time;

  joint_.initState();
  joint_.resetError();
  q1d = 0.0;
  qp1d = 0.0;
  qt1d = 0.0;
}

void EffortStateController::update(const ros::Time& time, const ros::Duration& period)
{
  // if ( (time.nsec % period.nsec) == 0)
  {
    joint_.updateState();
    evaluateDynamics();

    joint_.updateCommand(desired_state_, period);
    // joint_.resetCommand();
    joint_.addCompensationCommand(G);
    joint_.executeCommand();

    publishState(time);
  }
}

void EffortStateController::stopping(const ros::Time& time)
{

}

void EffortStateController::evaluateInertia()
{
  M = I133 + L2 * L2 * m1;
}

void EffortStateController::evaluateCoriolisCentripetal()
{
  C = 0;
}

void EffortStateController::evaluateGravity()
{
  G = L2 * g * m1 * (gy * cos(q1) + gz * sin(q1));
}

void EffortStateController::evaluateDynamics()
{
  evaluateInertia();
  evaluateCoriolisCentripetal();
  evaluateGravity();
}

void EffortStateController::publishState(const ros::Time& time)
{
  if (last_state_published_time_ + state_publish_period_ < time)
  {
    last_state_published_time_ += state_publish_period_;
    
    state_.header.stamp = time;
    
    state_.desired.positions[0] = q1d;
    state_.desired.velocities[0] = qp1d;
    state_.desired.effort[0] = qt1d;

    state_.actual.positions[0] = q1;
    state_.actual.velocities[0] = qp1;
    state_.actual.accelerations[0] = qpp1;
    state_.actual.effort[0] = qt1;

    state_.error.positions[0] = state_.desired.positions[0] - state_.actual.positions[0];
    state_.error.velocities[0] = state_.desired.velocities[0] - state_.actual.velocities[0];
    state_.error.effort[0] = state_.desired.effort[0] - state_.actual.effort[0];
    
    pub_state_.publish(state_);
  }
}

void EffortStateController::setConfigCallback(innfos_joint_controllers::EffortStateControllerPIDConfig &config, uint32_t level)
{
  if (level == 0)
  {

    Kpp_ = config.pos_p;
    Kpd_ = config.pos_d;
    Kpi_ = config.pos_i;
    joint_.setPositionControlParam(Kpp_, Kpi_, Kpd_);

    Kvp_ = config.vel_p;
    Kvd_ = config.vel_d;
    Kvi_ = config.vel_i;
    joint_.setVelocityControlParam(Kvp_, Kvi_, Kvd_);
    ROS_INFO("Set control parameter successfully");
  }

  if (level == 1)
  {
    joint_.setPositionControlLoop(config.pos_loop);
    joint_.setVelocityControlLoop(config.vel_loop);
    ROS_INFO("Set control mode successfully");
  }
}

bool EffortStateController::setCommandCallback(controller_msgs::SetCommand::Request& req,
                                               controller_msgs::SetCommand::Response& resp)
{
  if (req.command.command.values.size() != 1)
  {
    ROS_ERROR_STREAM("The command size " << req.command.command.values.size() << " is not equal to joint number " << 1);
    resp.success = false;
    resp.status_message = "Command size doesn't match";
    return true;
  }
  
  desired_state_[state_mode_] = req.command.command.values[0];

  resp.success = true;
  return true;
}

}