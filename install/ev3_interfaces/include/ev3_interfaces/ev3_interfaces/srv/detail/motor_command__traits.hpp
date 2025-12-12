// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ev3_interfaces:srv/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
#define EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ev3_interfaces/srv/detail/motor_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ev3_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MotorCommand_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: motor_id
  {
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << ", ";
  }

  // member: value1
  {
    out << "value1: ";
    rosidl_generator_traits::value_to_yaml(msg.value1, out);
    out << ", ";
  }

  // member: value2
  {
    out << "value2: ";
    rosidl_generator_traits::value_to_yaml(msg.value2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: motor_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << "\n";
  }

  // member: value1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value1: ";
    rosidl_generator_traits::value_to_yaml(msg.value1, out);
    out << "\n";
  }

  // member: value2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value2: ";
    rosidl_generator_traits::value_to_yaml(msg.value2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorCommand_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ev3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ev3_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ev3_interfaces::srv::MotorCommand_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ev3_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ev3_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ev3_interfaces::srv::MotorCommand_Request & msg)
{
  return ev3_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ev3_interfaces::srv::MotorCommand_Request>()
{
  return "ev3_interfaces::srv::MotorCommand_Request";
}

template<>
inline const char * name<ev3_interfaces::srv::MotorCommand_Request>()
{
  return "ev3_interfaces/srv/MotorCommand_Request";
}

template<>
struct has_fixed_size<ev3_interfaces::srv::MotorCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ev3_interfaces::srv::MotorCommand_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ev3_interfaces::srv::MotorCommand_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ev3_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MotorCommand_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorCommand_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ev3_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ev3_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ev3_interfaces::srv::MotorCommand_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ev3_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ev3_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ev3_interfaces::srv::MotorCommand_Response & msg)
{
  return ev3_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ev3_interfaces::srv::MotorCommand_Response>()
{
  return "ev3_interfaces::srv::MotorCommand_Response";
}

template<>
inline const char * name<ev3_interfaces::srv::MotorCommand_Response>()
{
  return "ev3_interfaces/srv/MotorCommand_Response";
}

template<>
struct has_fixed_size<ev3_interfaces::srv::MotorCommand_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ev3_interfaces::srv::MotorCommand_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ev3_interfaces::srv::MotorCommand_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ev3_interfaces::srv::MotorCommand>()
{
  return "ev3_interfaces::srv::MotorCommand";
}

template<>
inline const char * name<ev3_interfaces::srv::MotorCommand>()
{
  return "ev3_interfaces/srv/MotorCommand";
}

template<>
struct has_fixed_size<ev3_interfaces::srv::MotorCommand>
  : std::integral_constant<
    bool,
    has_fixed_size<ev3_interfaces::srv::MotorCommand_Request>::value &&
    has_fixed_size<ev3_interfaces::srv::MotorCommand_Response>::value
  >
{
};

template<>
struct has_bounded_size<ev3_interfaces::srv::MotorCommand>
  : std::integral_constant<
    bool,
    has_bounded_size<ev3_interfaces::srv::MotorCommand_Request>::value &&
    has_bounded_size<ev3_interfaces::srv::MotorCommand_Response>::value
  >
{
};

template<>
struct is_service<ev3_interfaces::srv::MotorCommand>
  : std::true_type
{
};

template<>
struct is_service_request<ev3_interfaces::srv::MotorCommand_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ev3_interfaces::srv::MotorCommand_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__TRAITS_HPP_
