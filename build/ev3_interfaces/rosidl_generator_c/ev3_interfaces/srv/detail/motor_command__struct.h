// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ev3_interfaces:srv/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__STRUCT_H_
#define EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__srv__MotorCommand_Request
{
  /// 指令名稱
  rosidl_runtime_c__String mode;
  /// 馬達ID (0~5)，或 -1 代表全體
  int32_t motor_id;
  /// 參數1 (duty/degree)
  float value1;
  /// 參數2 (speed)
  float value2;
} ev3_interfaces__srv__MotorCommand_Request;

// Struct for a sequence of ev3_interfaces__srv__MotorCommand_Request.
typedef struct ev3_interfaces__srv__MotorCommand_Request__Sequence
{
  ev3_interfaces__srv__MotorCommand_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__srv__MotorCommand_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__srv__MotorCommand_Response
{
  bool success;
  rosidl_runtime_c__String message;
} ev3_interfaces__srv__MotorCommand_Response;

// Struct for a sequence of ev3_interfaces__srv__MotorCommand_Response.
typedef struct ev3_interfaces__srv__MotorCommand_Response__Sequence
{
  ev3_interfaces__srv__MotorCommand_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__srv__MotorCommand_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__STRUCT_H_
