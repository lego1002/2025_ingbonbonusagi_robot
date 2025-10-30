// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ev3_interfaces:srv/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ev3_interfaces/srv/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ev3_interfaces
{

namespace srv
{

namespace builder
{

class Init_MotorCommand_Request_value2
{
public:
  explicit Init_MotorCommand_Request_value2(::ev3_interfaces::srv::MotorCommand_Request & msg)
  : msg_(msg)
  {}
  ::ev3_interfaces::srv::MotorCommand_Request value2(::ev3_interfaces::srv::MotorCommand_Request::_value2_type arg)
  {
    msg_.value2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::srv::MotorCommand_Request msg_;
};

class Init_MotorCommand_Request_value1
{
public:
  explicit Init_MotorCommand_Request_value1(::ev3_interfaces::srv::MotorCommand_Request & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_Request_value2 value1(::ev3_interfaces::srv::MotorCommand_Request::_value1_type arg)
  {
    msg_.value1 = std::move(arg);
    return Init_MotorCommand_Request_value2(msg_);
  }

private:
  ::ev3_interfaces::srv::MotorCommand_Request msg_;
};

class Init_MotorCommand_Request_motor_id
{
public:
  explicit Init_MotorCommand_Request_motor_id(::ev3_interfaces::srv::MotorCommand_Request & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_Request_value1 motor_id(::ev3_interfaces::srv::MotorCommand_Request::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_MotorCommand_Request_value1(msg_);
  }

private:
  ::ev3_interfaces::srv::MotorCommand_Request msg_;
};

class Init_MotorCommand_Request_mode
{
public:
  Init_MotorCommand_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_Request_motor_id mode(::ev3_interfaces::srv::MotorCommand_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotorCommand_Request_motor_id(msg_);
  }

private:
  ::ev3_interfaces::srv::MotorCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::srv::MotorCommand_Request>()
{
  return ev3_interfaces::srv::builder::Init_MotorCommand_Request_mode();
}

}  // namespace ev3_interfaces


namespace ev3_interfaces
{

namespace srv
{

namespace builder
{

class Init_MotorCommand_Response_message
{
public:
  explicit Init_MotorCommand_Response_message(::ev3_interfaces::srv::MotorCommand_Response & msg)
  : msg_(msg)
  {}
  ::ev3_interfaces::srv::MotorCommand_Response message(::ev3_interfaces::srv::MotorCommand_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::srv::MotorCommand_Response msg_;
};

class Init_MotorCommand_Response_success
{
public:
  Init_MotorCommand_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_Response_message success(::ev3_interfaces::srv::MotorCommand_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MotorCommand_Response_message(msg_);
  }

private:
  ::ev3_interfaces::srv::MotorCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::srv::MotorCommand_Response>()
{
  return ev3_interfaces::srv::builder::Init_MotorCommand_Response_success();
}

}  // namespace ev3_interfaces

#endif  // EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
