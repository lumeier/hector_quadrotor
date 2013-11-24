//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_QUADROTOR_CONTROLLER_HANDLES_H
#define HECTOR_QUADROTOR_CONTROLLER_HANDLES_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <hector_uav_msgs/MotorStatus.h>
#include <hector_uav_msgs/MotorCommand.h>

namespace hector_quadrotor_controller {

class QuadrotorInterface;

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;
using geometry_msgs::Twist;
using geometry_msgs::Vector3;
using geometry_msgs::Wrench;
using sensor_msgs::Imu;
using hector_uav_msgs::MotorStatus;
using hector_uav_msgs::MotorCommand;

template <class Derived, typename T>
class Handle_
{
public:
  typedef T ValueType;

  Handle_(const std::string& name, const std::string& field = std::string()) : interface_(0), name_(name), field_(field), value_(0) {}
  Handle_(QuadrotorInterface *interface, const std::string& name, const std::string& field = std::string()) : interface_(interface), name_(name), field_(field), value_(0) {}
  Handle_(QuadrotorInterface *interface, const ValueType *source, const std::string& name, const std::string& field = std::string()) : interface_(interface), name_(name), field_(field) { *this = source; }
  virtual ~Handle_() {}

  virtual const std::string& getName() const { return name_; }
  virtual const std::string& getField() const { return field_; }

  virtual bool connected() const { return get(); }
  virtual void reset() { value_ = 0; }

  Derived& operator=(const ValueType *source) { value_ = source; return static_cast<Derived &>(*this); }
  const ValueType *get() const { return value_; }
  const ValueType &operator*() const { return *value_; }

protected:
  QuadrotorInterface *interface_;
  const std::string name_;
  const std::string field_;
  const ValueType *value_;
};

class PoseHandle : public Handle_<PoseHandle, Pose>
{
public:
  typedef Handle_<PoseHandle, Pose> Base;
  using Base::operator=;

  PoseHandle() : Base("pose") {}
  PoseHandle(QuadrotorInterface *interface) : Base(interface, "pose") {}
  PoseHandle(QuadrotorInterface *interface, const Pose *pose) : Base(interface, pose, "pose") {}
  virtual ~PoseHandle() {}

  const ValueType& pose() const { return *get(); }

  void getEulerRPY(double &roll, double &pitch, double &yaw) const;
  double getYaw() const;
  Vector3 toBody(const Vector3& nav) const;
  Vector3 fromBody(const Vector3& nav) const;
};
typedef boost::shared_ptr<PoseHandle> PoseHandlePtr;

class TwistHandle : public Handle_<TwistHandle, Twist>
{
public:
  typedef Handle_<TwistHandle, Twist> Base;
  using Base::operator=;

  TwistHandle() : Base("twist") {}
  TwistHandle(QuadrotorInterface *interface) : Base(interface, "twist") {}
  TwistHandle(QuadrotorInterface *interface, const Twist *twist) : Base(interface, twist, "twist") {}
  virtual ~TwistHandle() {}

  const ValueType& twist() const { return *get(); }
};
typedef boost::shared_ptr<TwistHandle> TwistHandlePtr;

class AccelerationHandle : public Handle_<AccelerationHandle, Vector3>
{
public:
  typedef Handle_<AccelerationHandle, Vector3> Base;
  using Base::operator=;

  AccelerationHandle(QuadrotorInterface *interface, const Vector3 *acceleration) : Base(interface, acceleration, "acceleration") {}
  virtual ~AccelerationHandle() {}

  const ValueType& acceleration() const { return *get(); }
};
typedef boost::shared_ptr<AccelerationHandle> AccelerationHandlePtr;

class StateHandle : public PoseHandle, public TwistHandle
{
public:
  StateHandle() {}
  StateHandle(QuadrotorInterface *interface, const Pose *pose, const Twist *twist) : PoseHandle(interface, pose), TwistHandle(interface, twist) {}
  virtual ~StateHandle() {}

  virtual bool connected() const { return PoseHandle::connected() && TwistHandle::connected(); }
};
typedef boost::shared_ptr<PoseHandle> PoseHandlePtr;

class ImuHandle : public Handle_<ImuHandle, Imu>
{
public:
  typedef Handle_<ImuHandle, Imu> Base;
  using Base::operator=;

  ImuHandle() : Base("imu") {}
  ImuHandle(QuadrotorInterface *interface, const Imu *imu) : Base(interface, imu, "imu") {}
  virtual ~ImuHandle() {}

  const ValueType& imu() const { return *get(); }
};
typedef boost::shared_ptr<ImuHandle> ImuHandlePtr;

class MotorStatusHandle : public Handle_<MotorStatusHandle, MotorStatus>
{
public:
  typedef Handle_<MotorStatusHandle, MotorStatus> Base;
  using Base::operator=;

  MotorStatusHandle() : Base("motor_status") {}
  MotorStatusHandle(QuadrotorInterface *interface, const MotorStatus *motor_status) : Base(interface, motor_status, "motor_status") {}
  virtual ~MotorStatusHandle() {}

  const ValueType& motorStatus() const { return *get(); }
};
typedef boost::shared_ptr<MotorStatusHandle> MotorStatusHandlePtr;

class CommandHandle
{
public:
  CommandHandle() : interface_(0), new_value_(false) {}
  CommandHandle(QuadrotorInterface *interface, const std::string& name, const std::string& field) : interface_(interface), name_(name), field_(field), new_value_(false) {}
  virtual ~CommandHandle() {}

  virtual const std::string& getName() const { return name_; }
  virtual const std::string& getField() const { return field_; }
  virtual bool connected() const = 0;
  virtual void reset() {}

  bool enabled();
  bool start();
  void stop();
  bool disconnect();

  template <typename T> T* ownData(T* data) { my_.reset(data); return data; }

  template <typename Derived> bool connectFrom(const Derived& output) {
    Derived *me = dynamic_cast<Derived *>(this);
    if (!me) return false;
    ROS_DEBUG("Connected output port '%s (%p)' to input port '%s (%p)'", output.getName().c_str(), &output, me->getName().c_str(), me);
    return (*me = output.get()).connected();
  }

  template <typename Derived> bool connectTo(Derived& input) const {
    const Derived *me = dynamic_cast<const Derived *>(this);
    if (!me) return false;
    ROS_DEBUG("Connected output port '%s (%p)' to input port '%s (%p)'", me->getName().c_str(), me, input.getName().c_str(), &input);
    return (input = me->get()).connected();
  }

private:
  QuadrotorInterface *interface_;
  const std::string name_;
  const std::string field_;
  boost::shared_ptr<void> my_;

protected:
  mutable bool new_value_;
  bool wasNew() const { bool old = new_value_; new_value_ = false; return old; }
};
typedef boost::shared_ptr<CommandHandle> CommandHandlePtr;

template <class Derived, typename T, class Parent = CommandHandle>
class CommandHandle_ : public Parent
{
public:
  typedef T ValueType;

  CommandHandle_() : command_(0) {}
  CommandHandle_(const Parent &other) : Parent(other), command_(0) {}
  CommandHandle_(QuadrotorInterface *interface, const std::string& name, const std::string& field = std::string()) : Parent(interface, name, field), command_(0) {}
  virtual ~CommandHandle_() {}

  virtual bool connected() const { return get(); }
  virtual void reset() { command_ = 0; Parent::reset(); }

  using Parent::operator=;
  Derived& operator=(ValueType *source) { command_ = source; return static_cast<Derived &>(*this); }
  ValueType *get() const { return command_; }
  ValueType &operator*() const { return *command_; }

  ValueType& command() { return *get(); }
  const ValueType& getCommand() const { this->new_value_ = false; return *get(); }
  void setCommand(const ValueType& command) { this->new_value_ = true; *get() = command; }
  bool getCommand(ValueType& command) const { command = *get(); return this->wasNew(); }

  bool update(ValueType& command) const {
    if (!connected()) return false;
    command = getCommand();
    return true;
  }

protected:
  ValueType *command_;
};

class PoseCommandHandle : public CommandHandle_<PoseCommandHandle, Pose>
{
public:
  typedef CommandHandle_<PoseCommandHandle, Pose> Base;
  using Base::operator=;

  PoseCommandHandle() {}
  PoseCommandHandle(QuadrotorInterface *interface, const std::string& name, const std::string& field = std::string()) : Base(interface, name, field) {}
  PoseCommandHandle(Pose* command) { *this = command; }
  virtual ~PoseCommandHandle() {}
};
typedef boost::shared_ptr<PoseCommandHandle> PoseCommandHandlePtr;

class HorizontalPositionCommandHandle : public CommandHandle_<HorizontalPositionCommandHandle, Point, PoseCommandHandle>
{
public:
  typedef CommandHandle_<HorizontalPositionCommandHandle, Point, PoseCommandHandle> Base;
  using Base::operator=;

  HorizontalPositionCommandHandle() {}
  HorizontalPositionCommandHandle(const PoseCommandHandle& other) : Base(other) {}
  HorizontalPositionCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "position.xy") {}
  HorizontalPositionCommandHandle(Point* command) { *this = command; }
  virtual ~HorizontalPositionCommandHandle() {}

  virtual bool connected() const { return get(); }
  ValueType *get() const { return command_ ? command_ : (PoseCommandHandle::get() ? &(PoseCommandHandle::get()->position) : 0); }

  using Base::getCommand;
  virtual bool getCommand(double &x, double &y) const {
    x = get()->x;
    y = get()->y;
    return wasNew();
  }

  using Base::setCommand;
  virtual void setCommand(double x, double y)
  {
    this->new_value_ = true;
    get()->x = x;
    get()->y = y;
  }

  using Base::update;
  bool update(Pose& command) const {
    if (!connected()) return false;
    getCommand(command.position.x, command.position.y);
    return true;
  }

  void getError(const PoseHandle &pose, double &x, double &y) const;
};
typedef boost::shared_ptr<HorizontalPositionCommandHandle> HorizontalPositionCommandHandlePtr;

class HeightCommandHandle : public CommandHandle_<HeightCommandHandle, double, PoseCommandHandle>
{
public:
  typedef CommandHandle_<HeightCommandHandle, double, PoseCommandHandle> Base;
  using Base::operator=;

  HeightCommandHandle() {}
  HeightCommandHandle(const PoseCommandHandle& other) : Base(other) {}
  HeightCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "position.z") {}
  HeightCommandHandle(double *command) { *this = command; }
  virtual ~HeightCommandHandle() {}

  bool connected() const { return get(); }
  ValueType *get() const { return command_ ? command_ : (PoseCommandHandle::get() ? &(PoseCommandHandle::get()->position.z) : 0); }

  using Base::update;
  bool update(Pose& command) const {
    if (!connected()) return false;
    command.position.z = getCommand();
    return true;
  }

  double getError(const PoseHandle &pose) const;
};
typedef boost::shared_ptr<HeightCommandHandle> HeightCommandHandlePtr;

class HeadingCommandHandle : public CommandHandle_<HeadingCommandHandle, Quaternion, PoseCommandHandle>
{
public:
  typedef CommandHandle_<HeadingCommandHandle, Quaternion, PoseCommandHandle> Base;
  using Base::operator=;

  HeadingCommandHandle() {}
  HeadingCommandHandle(const PoseCommandHandle& other) : Base(other), scalar_(0) {}
  HeadingCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "orientation.yaw"), scalar_(0) {}
  HeadingCommandHandle(Quaternion *command) { *this = command; }
  virtual ~HeadingCommandHandle() {}

  virtual bool connected() const { return (command_ != 0) || get(); }
  ValueType *get() const { return command_ ? command_ : (PoseCommandHandle::get() ? &(PoseCommandHandle::get()->orientation) : 0); }

  using Base::getCommand;
  double getCommand() const;

  using Base::setCommand;
  void setCommand(double command);

  using Base::update;
  bool update(Pose& command) const;

  double getError(const PoseHandle &pose) const;

protected:
  double *scalar_;
};
typedef boost::shared_ptr<HeadingCommandHandle> HeadingCommandHandlePtr;

class TwistCommandHandle : public CommandHandle_<TwistCommandHandle, Twist>
{
public:
  typedef CommandHandle_<TwistCommandHandle, Twist> Base;
  using Base::operator=;

  TwistCommandHandle() {}
  TwistCommandHandle(QuadrotorInterface *interface, const std::string& name, const std::string& field = std::string()) : Base(interface, name, field) {}
  TwistCommandHandle(Twist* command) { *this = command; }
  virtual ~TwistCommandHandle() {}
};
typedef boost::shared_ptr<TwistCommandHandle> TwistCommandHandlePtr;

class HorizontalVelocityCommandHandle : public CommandHandle_<HorizontalVelocityCommandHandle, Vector3, TwistCommandHandle>
{
public:
  typedef CommandHandle_<HorizontalVelocityCommandHandle, Vector3, TwistCommandHandle> Base;
  using Base::operator=;

  HorizontalVelocityCommandHandle() {}
  HorizontalVelocityCommandHandle(const TwistCommandHandle& other) : Base(other) {}
  HorizontalVelocityCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "linear.xy") {}
  HorizontalVelocityCommandHandle(Vector3* command) { *this = command; }
  virtual ~HorizontalVelocityCommandHandle() {}

  virtual bool connected() const { return get(); }
  ValueType *get() const { return command_ ? command_ : (TwistCommandHandle::get() ? &(TwistCommandHandle::get()->linear) : 0); }

  using Base::getCommand;
  bool getCommand(double &x, double &y) const {
    x = get()->x;
    y = get()->y;
  }

  using Base::setCommand;
  void setCommand(double x, double y)
  {
    get()->x = x;
    get()->y = y;
  }

  using Base::update;
  bool update(Twist& command) const {
    if (!connected()) return false;
    getCommand(command.linear.x, command.linear.y);
    return true;
  }
};
typedef boost::shared_ptr<HorizontalVelocityCommandHandle> HorizontalVelocityCommandHandlePtr;

class VerticalVelocityCommandHandle : public CommandHandle_<VerticalVelocityCommandHandle, double, TwistCommandHandle>
{
public:
  typedef CommandHandle_<VerticalVelocityCommandHandle, double, TwistCommandHandle> Base;
  using Base::operator=;

  VerticalVelocityCommandHandle() {}
  VerticalVelocityCommandHandle(const TwistCommandHandle& other) : Base(other) {}
  VerticalVelocityCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "linear.z") {}
  VerticalVelocityCommandHandle(double* command) { *this = command; }
  virtual ~VerticalVelocityCommandHandle() {}

  virtual bool connected() const { return get(); }
  ValueType *get() const { return command_ ? command_ : (TwistCommandHandle::get() ? &(TwistCommandHandle::get()->linear.z) : 0); }

  using Base::update;
  bool update(Twist& command) const {
    if (!connected()) return false;
    command.linear.z = *get();
    return true;
  }
};
typedef boost::shared_ptr<VerticalVelocityCommandHandle> VerticalVelocityCommandHandlePtr;

class AngularVelocityCommandHandle : public CommandHandle_<AngularVelocityCommandHandle, double, TwistCommandHandle>
{
public:
  typedef CommandHandle_<AngularVelocityCommandHandle, double, TwistCommandHandle> Base;
  using Base::operator=;

  AngularVelocityCommandHandle() {}
  AngularVelocityCommandHandle(const TwistCommandHandle& other) : Base(other) {}
  AngularVelocityCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "angular.z") {}
  AngularVelocityCommandHandle(double* command) { *this = command; }
  virtual ~AngularVelocityCommandHandle() {}

  virtual bool connected() const { return get(); }
  ValueType *get() const { return command_ ? command_ : (TwistCommandHandle::get() ? &(TwistCommandHandle::get()->angular.z) : 0); }

  using Base::update;
  bool update(Twist& command) const {
    if (!connected()) return false;
    command.linear.z = *get();
    return true;
  }
};
typedef boost::shared_ptr<AngularVelocityCommandHandle> AngularVelocityCommandHandlePtr;

class WrenchCommandHandle : public CommandHandle_<WrenchCommandHandle, Wrench>
{
public:
  typedef CommandHandle_<WrenchCommandHandle, Wrench> Base;
  using Base::operator=;

  WrenchCommandHandle() {}
  WrenchCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name) {}
  virtual ~WrenchCommandHandle() {}
};
typedef boost::shared_ptr<WrenchCommandHandle> WrenchCommandHandlePtr;

class MotorCommandHandle : public CommandHandle_<MotorCommandHandle, MotorCommand>
{
public:
  typedef CommandHandle_<MotorCommandHandle, MotorCommand> Base;
  using Base::operator=;

  MotorCommandHandle() {}
  MotorCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name) {}
  virtual ~MotorCommandHandle() {}
};
typedef boost::shared_ptr<MotorCommandHandle> MotorCommandHandlePtr;

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_HANDLES_H