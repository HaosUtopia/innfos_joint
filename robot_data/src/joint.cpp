#include<robot_data/joint.h>

namespace robot_data
{

bool Joint::init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface)
{
  return init(joint_name, joint_type, joint_interface, ros::NodeHandle());
}

bool Joint::init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface, ros::NodeHandle nh)
{
  joint_handle_ = joint_interface->getHandle(joint_name);
  
  if (joint_type == "position")
  {
    joint_type_ = POSITION;
  }
  else if (joint_type == "velocity")
  {
    joint_type_ = VELOCITY;
  }
  else if (joint_type == "effort")
  {
    joint_type_ = EFFORT;
  }
  else
  {
    ROS_ERROR_STREAM("Unrecognized joint type: " << joint_type);
    joint_handle_ = hardware_interface::JointHandle();
    return false;
  }

  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", nh))
  {
    ROS_ERROR("Failed to parse urdf file from parameter 'robot_description'");
    joint_handle_ = hardware_interface::JointHandle();
    return false;
  }

  urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
  if (!joint_urdf)
  {
    ROS_ERROR_STREAM("Could not find joint " << joint_name << " in urdf");
    joint_handle_ = hardware_interface::JointHandle();
    return false;
  }

  is_continous_ = joint_urdf->type == urdf::Joint::CONTINUOUS;
  min_limit_[0] = joint_urdf->limits->lower;
  max_limit_[0] = joint_urdf->limits->upper;
  min_limit_[1] = -joint_urdf->limits->velocity;
  max_limit_[1] = joint_urdf->limits->velocity;
  min_limit_[2] = -joint_urdf->limits->effort;
  max_limit_[2] = joint_urdf->limits->effort;

  if (is_continous_)
  {
    min_limit_[0] = -std::numeric_limits<double>::infinity();
    max_limit_[0] = std::numeric_limits<double>::infinity();
  }

  return true;
}

bool Joint::init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface, bool is_continous, double min_pos, double max_pos, double max_vel, double max_eff)
{
  return init(joint_name, joint_type, joint_interface, is_continous, min_pos, max_pos, max_vel, max_eff, ros::NodeHandle());
}

bool Joint::init(const std::string& joint_name, const std::string& joint_type, hardware_interface::JointCommandInterface* joint_interface, bool is_continous, double min_pos, double max_pos, double max_vel, double max_eff, ros::NodeHandle nh)
{
  joint_handle_ = joint_interface->getHandle(joint_name);
  
  if (joint_type == "position")
  {
    joint_type_ = POSITION;
  }
  else if (joint_type == "velocity")
  {
    joint_type_ = VELOCITY;
  }
  else if (joint_type == "effort")
  {
    joint_type_ = EFFORT;
  }
  else
  {
    ROS_ERROR_STREAM("Unrecognized joint type: " << joint_type);
    joint_handle_ = hardware_interface::JointHandle();
    return false;
  }

  is_continous_ = is_continous;
  min_limit_[0] = min_pos;
  max_limit_[0] = max_pos;
  min_limit_[1] = -max_vel;
  max_limit_[1] = max_vel;
  min_limit_[2] = -max_eff;
  max_limit_[2] = max_eff;

  if (is_continous_)
  {
    min_limit_[0] = -std::numeric_limits<double>::infinity();
    max_limit_[0] = std::numeric_limits<double>::infinity();
  }

  return true;
}

void Joint::setPositionControlLoop(const bool& loop)
{
  if (joint_type_ <= POSITION)
  {
    pos_loop_ = loop;
  }
  else
  {
    ROS_WARN("The current joint control type doesn't have position loop");
  }
}

void Joint::setVelocityControlLoop(const bool& loop)
{
  if (joint_type_ <= VELOCITY)
  {
    vel_loop_ = loop;
  }
  else
  {
    ROS_WARN("The current joint control type doesn't have velocity loop");
  }
}

void Joint::setPositionControlParam(const double& Kp, const double& Ki, const double Kd)
{
  if (joint_type_ <= POSITION)
  {
    Kpp_ = Kp;
    Kpi_ = Ki;
    Kpd_ = Kd;
  }
  else
  {
    ROS_WARN("The current joint control type doesn't have position loop");
  }
}

void Joint::setVelocityControlParam(const double& Kp, const double& Ki, const double Kd)
{
  if (joint_type_ <= VELOCITY)
  {
    Kvp_ = Kp;
    Kvi_ = Ki;
    Kvd_ = Kd;
  }
  else
  {
    ROS_WARN("The current joint control type doesn't have velocity loop");
  }
}

void Joint::setErrorThreshold(const double& et)
{
  et_ = std::abs(et);
}

void Joint::resetError()
{
  epp_ = 0.0;
  epi_ = 0.0;
  epd_ = 0.0;

  evp_ = 0.0;
  evi_ = 0.0;
  evd_ = 0.0;
}

void Joint::initState()
{
  current_state_[0] = joint_handle_.getPosition();
  current_state_[1] = joint_handle_.getVelocity();
  current_state_[2] = joint_handle_.getEffort();
  current_state_[3] = 0.0;
}

void Joint::updateState()
{
  std::copy(current_state_, current_state_ + 4, last_state_);
  current_state_[0] = joint_handle_.getPosition();
  current_state_[1] = joint_handle_.getVelocity();
  current_state_[2] = joint_handle_.getEffort();
  current_state_[3] = (current_state_[1] * current_state_[1] - last_state_[1] * last_state_[1]) / (2.0 * current_state_[0] - last_state_[0]);
}

void Joint::updateCommand(const double& desired_state, const ros::Duration& period)
{
  
  reference_state_ = desired_state;

  double ep = std::max(std::min(std::max(std::min(reference_state_, max_limit_[joint_type_]), min_limit_[joint_type_]) - current_state_[joint_type_], et_), -et_);
  double ed = (ep - epp_) / period.toSec();
  
  if (period == ros::Duration(0.0) || std::isnan(ep) || std::isnan(ed))
  {
    joint_command_ = 0.0;
    return;
  }
  
  epp_ = ep;
  epd_ = ed;
  epi_ += epp_ * period.toSec();
  
  joint_command_ = Kpp_ * epp_ + Kpd_ * epd_ + Kpi_ * epi_;
}

void Joint::updateCommand(double* desired_state, const ros::Duration& period)
{
  joint_command_ = 0.0;
  reference_state_ = 0.0;

  if (period == ros::Duration(0.0))
  {
    ROS_ERROR("The period can not be zero");
    return;
  }

  if (pos_loop_)
  {
    reference_state_ = desired_state[0] + joint_command_;
    double epp = std::max(std::min(std::max(std::min(reference_state_, max_limit_[0]), min_limit_[0]) - current_state_[0], et_), -et_);
    double epd = (epp - epp_) / period.toSec();

    epp_ = epp;
    epd_ = epd;
    epi_ += epp_ * period.toSec();

    joint_command_ = Kpp_ * epp_ + Kpd_ * epd_ + Kpi_ * epi_;

    //ROS_INFO_STREAM("delta velocity: " << joint_command_);
  }

  if (vel_loop_)
  {
    //ROS_INFO_STREAM("actual velocity: " << current_state_[1]);
    reference_state_ = desired_state[1] + joint_command_;
    double evp = std::max(std::min(reference_state_, max_limit_[1]), min_limit_[1]) - current_state_[1];
    double evd = (evp - evp_) / period.toSec();

    evp_ = evp;
    evd_ = evd;
    evi_ += evp_ * period.toSec();

    joint_command_ = Kvp_ * evp_ + Kvd_ * evd_ + Kvi_ * evi_;

    //ROS_INFO_STREAM("delta effort: " << joint_command_);
  }

  if (eff_loop_)
  {
    reference_state_ = desired_state[2] + joint_command_;
    joint_command_ = std::max(std::min(reference_state_, max_limit_[2]), min_limit_[2]);
  }
}

void Joint::setCommand(const double& command)
{
  joint_command_ = std::max(std::min(command, max_limit_[joint_type_]), min_limit_[joint_type_]);
}

void Joint::resetCommand()
{
  joint_command_ = 0.0;
}

void Joint::addCompensationCommand(const double& compensation_command)
{
  joint_command_ += compensation_command;
}

void Joint::executeCommand()
{
  joint_handle_.setCommand(joint_command_);
}

const bool& Joint::isContinous() const
{
  return is_continous_;
}

const double& Joint::minPosition() const
{
  return min_limit_[0];
}

const double& Joint::maxPosition() const
{
  return max_limit_[0];
}

const double& Joint::maxVelocity() const
{
  return max_limit_[1];
}

const double& Joint::maxEffort() const
{
  return max_limit_[2];
}

const double& Joint::currentState() const
{
  return current_state_[joint_type_];
}

const double& Joint::position() const
{
  return current_state_[0];
}

const double& Joint::velocity() const
{
  return current_state_[1];
}

const double& Joint::acceleration() const
{
  return current_state_[3];
}

const double& Joint::effort() const
{
  return current_state_[2];
}

}
