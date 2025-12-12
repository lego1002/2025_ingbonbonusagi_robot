// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ev3_interfaces:action/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EV3_INTERFACES__ACTION__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
#define EV3_INTERFACES__ACTION__DETAIL__MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ev3_interfaces/action/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ev3_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorCommand_Goal_value2
{
public:
  explicit Init_MotorCommand_Goal_value2(::ev3_interfaces::action::MotorCommand_Goal & msg)
  : msg_(msg)
  {}
  ::ev3_interfaces::action::MotorCommand_Goal value2(::ev3_interfaces::action::MotorCommand_Goal::_value2_type arg)
  {
    msg_.value2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_Goal msg_;
};

class Init_MotorCommand_Goal_value1
{
public:
  explicit Init_MotorCommand_Goal_value1(::ev3_interfaces::action::MotorCommand_Goal & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_Goal_value2 value1(::ev3_interfaces::action::MotorCommand_Goal::_value1_type arg)
  {
    msg_.value1 = std::move(arg);
    return Init_MotorCommand_Goal_value2(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_Goal msg_;
};

class Init_MotorCommand_Goal_motor_id
{
public:
  explicit Init_MotorCommand_Goal_motor_id(::ev3_interfaces::action::MotorCommand_Goal & msg)
  : msg_(msg)
  {}
  Init_MotorCommand_Goal_value1 motor_id(::ev3_interfaces::action::MotorCommand_Goal::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_MotorCommand_Goal_value1(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_Goal msg_;
};

class Init_MotorCommand_Goal_mode
{
public:
  Init_MotorCommand_Goal_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_Goal_motor_id mode(::ev3_interfaces::action::MotorCommand_Goal::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MotorCommand_Goal_motor_id(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::action::MotorCommand_Goal>()
{
  return ev3_interfaces::action::builder::Init_MotorCommand_Goal_mode();
}

}  // namespace ev3_interfaces


namespace ev3_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorCommand_Result_message
{
public:
  explicit Init_MotorCommand_Result_message(::ev3_interfaces::action::MotorCommand_Result & msg)
  : msg_(msg)
  {}
  ::ev3_interfaces::action::MotorCommand_Result message(::ev3_interfaces::action::MotorCommand_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_Result msg_;
};

class Init_MotorCommand_Result_success
{
public:
  Init_MotorCommand_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_Result_message success(::ev3_interfaces::action::MotorCommand_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MotorCommand_Result_message(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::action::MotorCommand_Result>()
{
  return ev3_interfaces::action::builder::Init_MotorCommand_Result_success();
}

}  // namespace ev3_interfaces


namespace ev3_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorCommand_Feedback_status
{
public:
  Init_MotorCommand_Feedback_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ev3_interfaces::action::MotorCommand_Feedback status(::ev3_interfaces::action::MotorCommand_Feedback::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::action::MotorCommand_Feedback>()
{
  return ev3_interfaces::action::builder::Init_MotorCommand_Feedback_status();
}

}  // namespace ev3_interfaces


namespace ev3_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorCommand_SendGoal_Request_goal
{
public:
  explicit Init_MotorCommand_SendGoal_Request_goal(::ev3_interfaces::action::MotorCommand_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::ev3_interfaces::action::MotorCommand_SendGoal_Request goal(::ev3_interfaces::action::MotorCommand_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_SendGoal_Request msg_;
};

class Init_MotorCommand_SendGoal_Request_goal_id
{
public:
  Init_MotorCommand_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_SendGoal_Request_goal goal_id(::ev3_interfaces::action::MotorCommand_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MotorCommand_SendGoal_Request_goal(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::action::MotorCommand_SendGoal_Request>()
{
  return ev3_interfaces::action::builder::Init_MotorCommand_SendGoal_Request_goal_id();
}

}  // namespace ev3_interfaces


namespace ev3_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorCommand_SendGoal_Response_stamp
{
public:
  explicit Init_MotorCommand_SendGoal_Response_stamp(::ev3_interfaces::action::MotorCommand_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::ev3_interfaces::action::MotorCommand_SendGoal_Response stamp(::ev3_interfaces::action::MotorCommand_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_SendGoal_Response msg_;
};

class Init_MotorCommand_SendGoal_Response_accepted
{
public:
  Init_MotorCommand_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_SendGoal_Response_stamp accepted(::ev3_interfaces::action::MotorCommand_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MotorCommand_SendGoal_Response_stamp(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::action::MotorCommand_SendGoal_Response>()
{
  return ev3_interfaces::action::builder::Init_MotorCommand_SendGoal_Response_accepted();
}

}  // namespace ev3_interfaces


namespace ev3_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorCommand_GetResult_Request_goal_id
{
public:
  Init_MotorCommand_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ev3_interfaces::action::MotorCommand_GetResult_Request goal_id(::ev3_interfaces::action::MotorCommand_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::action::MotorCommand_GetResult_Request>()
{
  return ev3_interfaces::action::builder::Init_MotorCommand_GetResult_Request_goal_id();
}

}  // namespace ev3_interfaces


namespace ev3_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorCommand_GetResult_Response_result
{
public:
  explicit Init_MotorCommand_GetResult_Response_result(::ev3_interfaces::action::MotorCommand_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::ev3_interfaces::action::MotorCommand_GetResult_Response result(::ev3_interfaces::action::MotorCommand_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_GetResult_Response msg_;
};

class Init_MotorCommand_GetResult_Response_status
{
public:
  Init_MotorCommand_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_GetResult_Response_result status(::ev3_interfaces::action::MotorCommand_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MotorCommand_GetResult_Response_result(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::action::MotorCommand_GetResult_Response>()
{
  return ev3_interfaces::action::builder::Init_MotorCommand_GetResult_Response_status();
}

}  // namespace ev3_interfaces


namespace ev3_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorCommand_FeedbackMessage_feedback
{
public:
  explicit Init_MotorCommand_FeedbackMessage_feedback(::ev3_interfaces::action::MotorCommand_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::ev3_interfaces::action::MotorCommand_FeedbackMessage feedback(::ev3_interfaces::action::MotorCommand_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_FeedbackMessage msg_;
};

class Init_MotorCommand_FeedbackMessage_goal_id
{
public:
  Init_MotorCommand_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorCommand_FeedbackMessage_feedback goal_id(::ev3_interfaces::action::MotorCommand_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MotorCommand_FeedbackMessage_feedback(msg_);
  }

private:
  ::ev3_interfaces::action::MotorCommand_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::ev3_interfaces::action::MotorCommand_FeedbackMessage>()
{
  return ev3_interfaces::action::builder::Init_MotorCommand_FeedbackMessage_goal_id();
}

}  // namespace ev3_interfaces

#endif  // EV3_INTERFACES__ACTION__DETAIL__MOTOR_COMMAND__BUILDER_HPP_
