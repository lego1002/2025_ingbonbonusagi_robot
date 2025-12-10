// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from ev3_interfaces:action/MotorCommand.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "ev3_interfaces/action/detail/motor_command__struct.h"
#include "ev3_interfaces/action/detail/motor_command__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_Goal_type_support_ids_t;

static const _MotorCommand_Goal_type_support_ids_t _MotorCommand_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_Goal_type_support_symbol_names_t _MotorCommand_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_Goal)),
  }
};

typedef struct _MotorCommand_Goal_type_support_data_t
{
  void * data[2];
} _MotorCommand_Goal_type_support_data_t;

static _MotorCommand_Goal_type_support_data_t _MotorCommand_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_Goal_message_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorCommand_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_Goal)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_Result_type_support_ids_t;

static const _MotorCommand_Result_type_support_ids_t _MotorCommand_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_Result_type_support_symbol_names_t _MotorCommand_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_Result)),
  }
};

typedef struct _MotorCommand_Result_type_support_data_t
{
  void * data[2];
} _MotorCommand_Result_type_support_data_t;

static _MotorCommand_Result_type_support_data_t _MotorCommand_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_Result_message_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_Result_message_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_Result_message_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorCommand_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_Result)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_Feedback_type_support_ids_t;

static const _MotorCommand_Feedback_type_support_ids_t _MotorCommand_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_Feedback_type_support_symbol_names_t _MotorCommand_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_Feedback)),
  }
};

typedef struct _MotorCommand_Feedback_type_support_data_t
{
  void * data[2];
} _MotorCommand_Feedback_type_support_data_t;

static _MotorCommand_Feedback_type_support_data_t _MotorCommand_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_Feedback_message_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorCommand_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_Feedback)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_SendGoal_Request_type_support_ids_t;

static const _MotorCommand_SendGoal_Request_type_support_ids_t _MotorCommand_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_SendGoal_Request_type_support_symbol_names_t _MotorCommand_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_SendGoal_Request)),
  }
};

typedef struct _MotorCommand_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _MotorCommand_SendGoal_Request_type_support_data_t;

static _MotorCommand_SendGoal_Request_type_support_data_t _MotorCommand_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_SendGoal_Request_message_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorCommand_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_SendGoal_Request)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_SendGoal_Response_type_support_ids_t;

static const _MotorCommand_SendGoal_Response_type_support_ids_t _MotorCommand_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_SendGoal_Response_type_support_symbol_names_t _MotorCommand_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_SendGoal_Response)),
  }
};

typedef struct _MotorCommand_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _MotorCommand_SendGoal_Response_type_support_data_t;

static _MotorCommand_SendGoal_Response_type_support_data_t _MotorCommand_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_SendGoal_Response_message_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorCommand_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_SendGoal_Response)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_SendGoal_type_support_ids_t;

static const _MotorCommand_SendGoal_type_support_ids_t _MotorCommand_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_SendGoal_type_support_symbol_names_t _MotorCommand_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_SendGoal)),
  }
};

typedef struct _MotorCommand_SendGoal_type_support_data_t
{
  void * data[2];
} _MotorCommand_SendGoal_type_support_data_t;

static _MotorCommand_SendGoal_type_support_data_t _MotorCommand_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_SendGoal_service_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MotorCommand_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_SendGoal)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_GetResult_Request_type_support_ids_t;

static const _MotorCommand_GetResult_Request_type_support_ids_t _MotorCommand_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_GetResult_Request_type_support_symbol_names_t _MotorCommand_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_GetResult_Request)),
  }
};

typedef struct _MotorCommand_GetResult_Request_type_support_data_t
{
  void * data[2];
} _MotorCommand_GetResult_Request_type_support_data_t;

static _MotorCommand_GetResult_Request_type_support_data_t _MotorCommand_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_GetResult_Request_message_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorCommand_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_GetResult_Request)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_GetResult_Response_type_support_ids_t;

static const _MotorCommand_GetResult_Response_type_support_ids_t _MotorCommand_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_GetResult_Response_type_support_symbol_names_t _MotorCommand_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_GetResult_Response)),
  }
};

typedef struct _MotorCommand_GetResult_Response_type_support_data_t
{
  void * data[2];
} _MotorCommand_GetResult_Response_type_support_data_t;

static _MotorCommand_GetResult_Response_type_support_data_t _MotorCommand_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_GetResult_Response_message_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorCommand_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_GetResult_Response)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_GetResult_type_support_ids_t;

static const _MotorCommand_GetResult_type_support_ids_t _MotorCommand_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_GetResult_type_support_symbol_names_t _MotorCommand_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_GetResult)),
  }
};

typedef struct _MotorCommand_GetResult_type_support_data_t
{
  void * data[2];
} _MotorCommand_GetResult_type_support_data_t;

static _MotorCommand_GetResult_type_support_data_t _MotorCommand_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_GetResult_service_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MotorCommand_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_GetResult)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__struct.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace ev3_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MotorCommand_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MotorCommand_FeedbackMessage_type_support_ids_t;

static const _MotorCommand_FeedbackMessage_type_support_ids_t _MotorCommand_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MotorCommand_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MotorCommand_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MotorCommand_FeedbackMessage_type_support_symbol_names_t _MotorCommand_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ev3_interfaces, action, MotorCommand_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ev3_interfaces, action, MotorCommand_FeedbackMessage)),
  }
};

typedef struct _MotorCommand_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _MotorCommand_FeedbackMessage_type_support_data_t;

static _MotorCommand_FeedbackMessage_type_support_data_t _MotorCommand_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MotorCommand_FeedbackMessage_message_typesupport_map = {
  2,
  "ev3_interfaces",
  &_MotorCommand_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_MotorCommand_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_MotorCommand_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MotorCommand_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MotorCommand_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace ev3_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_FeedbackMessage)() {
  return &::ev3_interfaces::action::rosidl_typesupport_c::MotorCommand_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "ev3_interfaces/action/motor_command.h"
// already included above
// #include "ev3_interfaces/action/detail/motor_command__type_support.h"

static rosidl_action_type_support_t _ev3_interfaces__action__MotorCommand__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, ev3_interfaces, action, MotorCommand)()
{
  // Thread-safe by always writing the same values to the static struct
  _ev3_interfaces__action__MotorCommand__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_SendGoal)();
  _ev3_interfaces__action__MotorCommand__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_GetResult)();
  _ev3_interfaces__action__MotorCommand__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _ev3_interfaces__action__MotorCommand__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, ev3_interfaces, action, MotorCommand_FeedbackMessage)();
  _ev3_interfaces__action__MotorCommand__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_ev3_interfaces__action__MotorCommand__typesupport_c;
}

#ifdef __cplusplus
}
#endif
