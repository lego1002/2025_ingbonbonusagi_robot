// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ev3_interfaces:action/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EV3_INTERFACES__ACTION__DETAIL__MOTOR_COMMAND__STRUCT_H_
#define EV3_INTERFACES__ACTION__DETAIL__MOTOR_COMMAND__STRUCT_H_

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

/// Struct defined in action/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__action__MotorCommand_Goal
{
  rosidl_runtime_c__String mode;
  int32_t motor_id;
  float value1;
  float value2;
} ev3_interfaces__action__MotorCommand_Goal;

// Struct for a sequence of ev3_interfaces__action__MotorCommand_Goal.
typedef struct ev3_interfaces__action__MotorCommand_Goal__Sequence
{
  ev3_interfaces__action__MotorCommand_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__action__MotorCommand_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__action__MotorCommand_Result
{
  bool success;
  rosidl_runtime_c__String message;
} ev3_interfaces__action__MotorCommand_Result;

// Struct for a sequence of ev3_interfaces__action__MotorCommand_Result.
typedef struct ev3_interfaces__action__MotorCommand_Result__Sequence
{
  ev3_interfaces__action__MotorCommand_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__action__MotorCommand_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__action__MotorCommand_Feedback
{
  rosidl_runtime_c__String status;
} ev3_interfaces__action__MotorCommand_Feedback;

// Struct for a sequence of ev3_interfaces__action__MotorCommand_Feedback.
typedef struct ev3_interfaces__action__MotorCommand_Feedback__Sequence
{
  ev3_interfaces__action__MotorCommand_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__action__MotorCommand_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "ev3_interfaces/action/detail/motor_command__struct.h"

/// Struct defined in action/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__action__MotorCommand_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  ev3_interfaces__action__MotorCommand_Goal goal;
} ev3_interfaces__action__MotorCommand_SendGoal_Request;

// Struct for a sequence of ev3_interfaces__action__MotorCommand_SendGoal_Request.
typedef struct ev3_interfaces__action__MotorCommand_SendGoal_Request__Sequence
{
  ev3_interfaces__action__MotorCommand_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__action__MotorCommand_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__action__MotorCommand_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} ev3_interfaces__action__MotorCommand_SendGoal_Response;

// Struct for a sequence of ev3_interfaces__action__MotorCommand_SendGoal_Response.
typedef struct ev3_interfaces__action__MotorCommand_SendGoal_Response__Sequence
{
  ev3_interfaces__action__MotorCommand_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__action__MotorCommand_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__action__MotorCommand_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} ev3_interfaces__action__MotorCommand_GetResult_Request;

// Struct for a sequence of ev3_interfaces__action__MotorCommand_GetResult_Request.
typedef struct ev3_interfaces__action__MotorCommand_GetResult_Request__Sequence
{
  ev3_interfaces__action__MotorCommand_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__action__MotorCommand_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "ev3_interfaces/action/detail/motor_command__struct.h"

/// Struct defined in action/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__action__MotorCommand_GetResult_Response
{
  int8_t status;
  ev3_interfaces__action__MotorCommand_Result result;
} ev3_interfaces__action__MotorCommand_GetResult_Response;

// Struct for a sequence of ev3_interfaces__action__MotorCommand_GetResult_Response.
typedef struct ev3_interfaces__action__MotorCommand_GetResult_Response__Sequence
{
  ev3_interfaces__action__MotorCommand_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__action__MotorCommand_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "ev3_interfaces/action/detail/motor_command__struct.h"

/// Struct defined in action/MotorCommand in the package ev3_interfaces.
typedef struct ev3_interfaces__action__MotorCommand_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  ev3_interfaces__action__MotorCommand_Feedback feedback;
} ev3_interfaces__action__MotorCommand_FeedbackMessage;

// Struct for a sequence of ev3_interfaces__action__MotorCommand_FeedbackMessage.
typedef struct ev3_interfaces__action__MotorCommand_FeedbackMessage__Sequence
{
  ev3_interfaces__action__MotorCommand_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ev3_interfaces__action__MotorCommand_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EV3_INTERFACES__ACTION__DETAIL__MOTOR_COMMAND__STRUCT_H_
