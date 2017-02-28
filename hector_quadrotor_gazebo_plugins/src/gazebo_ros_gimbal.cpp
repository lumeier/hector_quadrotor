
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
  printf("Loading Gimbal!");
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
    else if (current->GetScopedName().find("roll") != std::string::npos)
    {
      std::cout << "roll" << std::endl;
      _roll_joint = current;
    }
    else if (current->GetScopedName().find("prop1_joint") != std::string::npos)
    {
      std::cout << "prop1" << std::endl;
      _prop1_joint = current;
    }
    else if (current->GetScopedName().find("prop2_joint") != std::string::npos)
    {
      std::cout << "prop2" << std::endl;
      _prop2_joint = current;
    }
    else if (current->GetScopedName().find("prop3_joint") != std::string::npos)
    {
      std::cout << "prop3" << std::endl;
      _prop3_joint = current;
    }
    else if (current->GetScopedName().find("prop4_joint") != std::string::npos)
    {
      std::cout << "prop4" << std::endl;
      _prop4_joint = current;
    }
  }

  // // Setup a P-controller, with a gain of 0.5.
  // _pan_pid = common::PID(2, 1, 0);
  // _tilt_pid = common::PID(2, 1, 0);
  // _roll_pid = common::PID(2, 1, 0);
  //
  // // Apply the P-controller to the joint.
  // _model->GetJointController()->SetPositionPID(
  //     _pan_joint->GetScopedName(), _pan_pid);
  // _model->GetJointController()->SetPositionPID(
  //     _tilt_joint->GetScopedName(), _tilt_pid);
  // _model->GetJointController()->SetPositionPID(
  //     _roll_joint->GetScopedName(), _roll_pid);

  // Set the joint's target velocity. This target velocity is just
  // for demonstration purposes.
  //_model->GetJointController()->SetPositionTarget(
    //  _pan_joint->GetScopedName(), 0);
  //_model->GetJointController()->SetPositionTarget(
      //_tilt_joint->GetScopedName(), 0);

  // ROS
  _nh = new ros::NodeHandle(_namespace);
  _pan_sub = _nh->subscribe("/pan_command", 1, &GazeboRosGimbal::pan_callback, this);
  _tilt_sub = _nh->subscribe("/tilt_command", 1, &GazeboRosGimbal::tilt_callback, this);
  _euler_sub = _nh->subscribe("/ground_truth_to_tf/euler", 1, &GazeboRosGimbal::euler_callback, this);
  _motor_sub = _nh->subscribe("/mot_anim_toggle", 1, &GazeboRosGimbal::anim_callback, this);
}

void GazeboRosGimbal::pan_callback(const std_msgs::Float64::ConstPtr& msg)
{
  _pan_value = msg->data;
}

void GazeboRosGimbal::tilt_callback(const std_msgs::Float64::ConstPtr& msg)
{
 _tilt_value = msg->data;
}


void GazeboRosGimbal::euler_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  _model->GetJointController()->SetJointPosition(
      _roll_joint, -msg->vector.x);

  _model->GetJointController()->SetJointPosition(
      _tilt_joint, -msg->vector.y + _tilt_value);

  _model->GetJointController()->SetJointPosition(
      _pan_joint, -msg->vector.z + _pan_value);
}
//Hack to animate propellers
void GazeboRosGimbal::anim_callback(const std_msgs::Int32::ConstPtr& msg)
{
  if (msg->data==1)
    {
      printf("Start Motor animation!\n");
      _prop1_joint->SetForce(2,-0.05);
      _prop2_joint->SetForce(2,0.05);
      _prop3_joint->SetForce(2,0.05);
      _prop4_joint->SetForce(2,-0.05);
    }
  else if (msg->data==0)
    {
      printf("Stop Motor animation!\n");
      _prop1_joint->SetForce(2,0.05);
      _prop2_joint->SetForce(2,-0.05);
      _prop3_joint->SetForce(2,-0.05);
      _prop4_joint->SetForce(2,0.05);
    }
}

void GazeboRosGimbal::Reset()
{
}

void GazeboRosGimbal::Update()
{
  printf("Gimbal Update!\n");
}
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGimbal)

} // namespace gazebo
