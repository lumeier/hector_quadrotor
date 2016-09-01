
#include <hector_quadrotor_gazebo_plugins/gazebo_ros_gimbal.h>

namespace gazebo {
  
GazeboRosGimbal::GazeboRosGimbal()
  : _velocity(0.0)
{
}

GazeboRosGimbal::~GazeboRosGimbal()
{
  _nh->shutdown();
  delete _nh;
}
////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGimbal::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Safety check
  if (model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
    return;
  }

  // load parameters
  if (sdf->HasElement("robotNamespace")) _namespace = sdf->GetElement("robotNamespace")->Get<std::string>();

  // Store the model pointer for convenience.
  _model = model;

  // Get the first joint. We are making an assumption about the model
  // having one joint that is the rotational joint.
  _joint = _model->GetJoints()[0];

  // Setup a P-controller, with a gain of 0.5.
  _pid = common::PID(0.01, 0, 0);

  // Apply the P-controller to the joint.
  _model->GetJointController()->SetPositionPID(
      _joint->GetScopedName(), _pid);

  // Set the joint's target velocity. This target velocity is just
  // for demonstration purposes.
  _model->GetJointController()->SetPositionTarget(
      _joint->GetScopedName(), 0.0);

  // ROS
  _nh = new ros::NodeHandle(_namespace);
  _cmd_sub = _nh->subscribe("/gimbal_command", 1, &GazeboRosGimbal::cmd_callback, this);
}

void GazeboRosGimbal::cmd_callback(const std_msgs::Float64::ConstPtr& msg)
{
  _model->GetJointController()->SetPositionTarget(
      _joint->GetScopedName(), msg->data);
}

void GazeboRosGimbal::Reset()
{
}

void GazeboRosGimbal::Update()
{
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGimbal)

} // namespace gazebo
