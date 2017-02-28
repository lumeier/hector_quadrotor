#ifndef _GAZEBO_PLUGINS_GAZEBO_ROS_GIMBAL_H_
#define _GAZEBO_PLUGINS_GAZEBO_ROS_GIMBAL_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3Stamped.h>

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
    ros::Subscriber _euler_sub;
    ros::Subscriber _motor_sub;
    /// \brief Pointer to the model.
    physics::ModelPtr _model;

    /// \brief Pointer to the joint.
    physics::JointPtr _pan_joint;
    physics::JointPtr _tilt_joint;
    physics::JointPtr _roll_joint;

    physics::JointPtr _prop1_joint;
    physics::JointPtr _prop2_joint;
    physics::JointPtr _prop3_joint;
    physics::JointPtr _prop4_joint;

    /// \brief A PID controller for the joint.
    common::PID _pan_pid;
    common::PID _tilt_pid;
    common::PID _roll_pid;
    double _pan_value = 0;
    double _tilt_value = 0;

    std::string _namespace;

    void pan_callback(const std_msgs::Float64::ConstPtr& msg);
    void tilt_callback(const std_msgs::Float64::ConstPtr& msg);
    void euler_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void anim_callback(const std_msgs::Int32::ConstPtr& msg);
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GIMBAL_H_
