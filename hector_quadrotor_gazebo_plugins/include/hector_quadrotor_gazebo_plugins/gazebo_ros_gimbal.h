#ifndef _GAZEBO_PLUGINS_GAZEBO_ROS_GIMBAL_H_
#define _GAZEBO_PLUGINS_GAZEBO_ROS_GIMBAL_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{

class GazeboRosGimbal : public ModelPlugin
{
public:
  GazeboRosGimbal();
  virtual ~GazeboRosGimbal();

protected:
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void Reset();
  virtual void Update();

private:
    
    ros::NodeHandle* _nh;
    ros::Subscriber _pan_sub;
    ros::Subscriber _tilt_sub;

    /// \brief Pointer to the model.
    physics::ModelPtr _model;

    /// \brief Pointer to the joint.
    physics::JointPtr _pan_joint;
    physics::JointPtr _tilt_joint;

    /// \brief A PID controller for the joint.
    common::PID _pan_pid;
    common::PID _tilt_pid;

    std::string _namespace;

    void pan_callback(const std_msgs::Float64::ConstPtr& msg);
    void tilt_callback(const std_msgs::Float64::ConstPtr& msg);
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GIMBAL_H_
