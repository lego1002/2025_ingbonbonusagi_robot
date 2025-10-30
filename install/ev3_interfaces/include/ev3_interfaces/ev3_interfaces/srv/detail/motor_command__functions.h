// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from ev3_interfaces:srv/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__FUNCTIONS_H_
#define EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "ev3_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "ev3_interfaces/srv/detail/motor_command__struct.h"

/// Initialize srv/MotorCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ev3_interfaces__srv__MotorCommand_Request
 * )) before or use
 * ev3_interfaces__srv__MotorCommand_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Request__init(ev3_interfaces__srv__MotorCommand_Request * msg);

/// Finalize srv/MotorCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
void
ev3_interfaces__srv__MotorCommand_Request__fini(ev3_interfaces__srv__MotorCommand_Request * msg);

/// Create srv/MotorCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ev3_interfaces__srv__MotorCommand_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
ev3_interfaces__srv__MotorCommand_Request *
ev3_interfaces__srv__MotorCommand_Request__create();

/// Destroy srv/MotorCommand message.
/**
 * It calls
 * ev3_interfaces__srv__MotorCommand_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
void
ev3_interfaces__srv__MotorCommand_Request__destroy(ev3_interfaces__srv__MotorCommand_Request * msg);

/// Check for srv/MotorCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Request__are_equal(const ev3_interfaces__srv__MotorCommand_Request * lhs, const ev3_interfaces__srv__MotorCommand_Request * rhs);

/// Copy a srv/MotorCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Request__copy(
  const ev3_interfaces__srv__MotorCommand_Request * input,
  ev3_interfaces__srv__MotorCommand_Request * output);

/// Initialize array of srv/MotorCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * ev3_interfaces__srv__MotorCommand_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Request__Sequence__init(ev3_interfaces__srv__MotorCommand_Request__Sequence * array, size_t size);

/// Finalize array of srv/MotorCommand messages.
/**
 * It calls
 * ev3_interfaces__srv__MotorCommand_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
void
ev3_interfaces__srv__MotorCommand_Request__Sequence__fini(ev3_interfaces__srv__MotorCommand_Request__Sequence * array);

/// Create array of srv/MotorCommand messages.
/**
 * It allocates the memory for the array and calls
 * ev3_interfaces__srv__MotorCommand_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
ev3_interfaces__srv__MotorCommand_Request__Sequence *
ev3_interfaces__srv__MotorCommand_Request__Sequence__create(size_t size);

/// Destroy array of srv/MotorCommand messages.
/**
 * It calls
 * ev3_interfaces__srv__MotorCommand_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
void
ev3_interfaces__srv__MotorCommand_Request__Sequence__destroy(ev3_interfaces__srv__MotorCommand_Request__Sequence * array);

/// Check for srv/MotorCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Request__Sequence__are_equal(const ev3_interfaces__srv__MotorCommand_Request__Sequence * lhs, const ev3_interfaces__srv__MotorCommand_Request__Sequence * rhs);

/// Copy an array of srv/MotorCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Request__Sequence__copy(
  const ev3_interfaces__srv__MotorCommand_Request__Sequence * input,
  ev3_interfaces__srv__MotorCommand_Request__Sequence * output);

/// Initialize srv/MotorCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * ev3_interfaces__srv__MotorCommand_Response
 * )) before or use
 * ev3_interfaces__srv__MotorCommand_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Response__init(ev3_interfaces__srv__MotorCommand_Response * msg);

/// Finalize srv/MotorCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
void
ev3_interfaces__srv__MotorCommand_Response__fini(ev3_interfaces__srv__MotorCommand_Response * msg);

/// Create srv/MotorCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * ev3_interfaces__srv__MotorCommand_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
ev3_interfaces__srv__MotorCommand_Response *
ev3_interfaces__srv__MotorCommand_Response__create();

/// Destroy srv/MotorCommand message.
/**
 * It calls
 * ev3_interfaces__srv__MotorCommand_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
void
ev3_interfaces__srv__MotorCommand_Response__destroy(ev3_interfaces__srv__MotorCommand_Response * msg);

/// Check for srv/MotorCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Response__are_equal(const ev3_interfaces__srv__MotorCommand_Response * lhs, const ev3_interfaces__srv__MotorCommand_Response * rhs);

/// Copy a srv/MotorCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Response__copy(
  const ev3_interfaces__srv__MotorCommand_Response * input,
  ev3_interfaces__srv__MotorCommand_Response * output);

/// Initialize array of srv/MotorCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * ev3_interfaces__srv__MotorCommand_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Response__Sequence__init(ev3_interfaces__srv__MotorCommand_Response__Sequence * array, size_t size);

/// Finalize array of srv/MotorCommand messages.
/**
 * It calls
 * ev3_interfaces__srv__MotorCommand_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
void
ev3_interfaces__srv__MotorCommand_Response__Sequence__fini(ev3_interfaces__srv__MotorCommand_Response__Sequence * array);

/// Create array of srv/MotorCommand messages.
/**
 * It allocates the memory for the array and calls
 * ev3_interfaces__srv__MotorCommand_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
ev3_interfaces__srv__MotorCommand_Response__Sequence *
ev3_interfaces__srv__MotorCommand_Response__Sequence__create(size_t size);

/// Destroy array of srv/MotorCommand messages.
/**
 * It calls
 * ev3_interfaces__srv__MotorCommand_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
void
ev3_interfaces__srv__MotorCommand_Response__Sequence__destroy(ev3_interfaces__srv__MotorCommand_Response__Sequence * array);

/// Check for srv/MotorCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Response__Sequence__are_equal(const ev3_interfaces__srv__MotorCommand_Response__Sequence * lhs, const ev3_interfaces__srv__MotorCommand_Response__Sequence * rhs);

/// Copy an array of srv/MotorCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_ev3_interfaces
bool
ev3_interfaces__srv__MotorCommand_Response__Sequence__copy(
  const ev3_interfaces__srv__MotorCommand_Response__Sequence * input,
  ev3_interfaces__srv__MotorCommand_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__FUNCTIONS_H_
