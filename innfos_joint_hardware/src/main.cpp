#include <innfos_joint_hardware/innfos_joint_hardware.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "innfos_joint_hardware");
  ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
  ros::NodeHandle hw_nh("innfos");
  hw_nh.setCallbackQueue(&ros_queue);
  JointRobotHardware joint;

  if (!joint.init(nh, hw_nh))
  {
    ROS_ERROR("Failed to initailize the innfos joint hardware");
    return 1;
  }

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin(&ros_queue);

  return 0;
}
