
#include <hector_quadrotor_gazebo_plugins/gazebo_ros_gimbal.h>

namespace gazebo {
  
GazeboRosGimbal::GazeboRosGimbal()
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
  for (unsigned int i = 0; i < _model->GetJointCount(); ++i)
  {
    physics::JointPtr current = _model->GetJoints()[i];
    std::cout << current->GetScopedName() << std::endl;
    if (current->GetScopedName().find("pan") != std::string::npos)
    {
      std::cout << "pan" << std::endl;
      _pan_joint = current;
    }
    else if (current->GetScopedName().find("tilt") != std::string::npos)
    {
      std::cout << "tilt" << std::endl;
      _tilt_joint = current;
    }
  }

  // Setup a P-controller, with a gain of 0.5.
  _pan_pid = common::PID(0.01, 0, 0);
  _tilt_pid = common::PID(0.01, 0, 0);

  // Apply the P-controller to the joint.
  _model->GetJointController()->SetPositionPID(
      _pan_joint->GetScopedName(), _pan_pid);
  _model->GetJointController()->SetPositionPID(
      _tilt_joint->GetScopedName(), _tilt_pid);

  // Set the joint's target velocity. This target velocity is just
  // for demonstration purposes.
  _model->GetJointController()->SetPositionTarget(
      _pan_joint->GetScopedName(), 0.0);
  _model->GetJointController()->SetPositionTarget(
      _tilt_joint->GetScopedName(), 0.0);

  // ROS
  _nh = new ros::NodeHandle(_namespace);
  _pan_sub = _nh->subscribe("/pan_command", 1, &GazeboRosGimbal::pan_callback, this);
  _tilt_sub = _nh->subscribe("/tilt_command", 1, &GazeboRosGimbal::tilt_callback, this);
}

void GazeboRosGimbal::pan_callback(const std_msgs::Float64::ConstPtr& msg)
{
  _model->GetJointController()->SetPositionTarget(
      _pan_joint->GetScopedName(), msg->data);
}

void GazeboRosGimbal::tilt_callback(const std_msgs::Float64::ConstPtr& msg)
{
  _model->GetJointController()->SetPositionTarget(
      _tilt_joint->GetScopedName(), msg->data);
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
