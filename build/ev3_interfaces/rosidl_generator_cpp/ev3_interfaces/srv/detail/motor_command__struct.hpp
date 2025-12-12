// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ev3_interfaces:srv/MotorCommand.idl
// generated code does not contain a copyright notice

#ifndef EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
#define EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ev3_interfaces__srv__MotorCommand_Request __attribute__((deprecated))
#else
# define DEPRECATED__ev3_interfaces__srv__MotorCommand_Request __declspec(deprecated)
#endif

namespace ev3_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotorCommand_Request_
{
  using Type = MotorCommand_Request_<ContainerAllocator>;

  explicit MotorCommand_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = "";
      this->motor_id = 0l;
      this->value1 = 0.0f;
      this->value2 = 0.0f;
    }
  }

  explicit MotorCommand_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : mode(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = "";
      this->motor_id = 0l;
      this->value1 = 0.0f;
      this->value2 = 0.0f;
    }
  }

  // field types and members
  using _mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mode_type mode;
  using _motor_id_type =
    int32_t;
  _motor_id_type motor_id;
  using _value1_type =
    float;
  _value1_type value1;
  using _value2_type =
    float;
  _value2_type value2;

  // setters for named parameter idiom
  Type & set__mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__motor_id(
    const int32_t & _arg)
  {
    this->motor_id = _arg;
    return *this;
  }
  Type & set__value1(
    const float & _arg)
  {
    this->value1 = _arg;
    return *this;
  }
  Type & set__value2(
    const float & _arg)
  {
    this->value2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ev3_interfaces__srv__MotorCommand_Request
    std::shared_ptr<ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ev3_interfaces__srv__MotorCommand_Request
    std::shared_ptr<ev3_interfaces::srv::MotorCommand_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommand_Request_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->value1 != other.value1) {
      return false;
    }
    if (this->value2 != other.value2) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommand_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommand_Request_

// alias to use template instance with default allocator
using MotorCommand_Request =
  ev3_interfaces::srv::MotorCommand_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ev3_interfaces


#ifndef _WIN32
# define DEPRECATED__ev3_interfaces__srv__MotorCommand_Response __attribute__((deprecated))
#else
# define DEPRECATED__ev3_interfaces__srv__MotorCommand_Response __declspec(deprecated)
#endif

namespace ev3_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MotorCommand_Response_
{
  using Type = MotorCommand_Response_<ContainerAllocator>;

  explicit MotorCommand_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit MotorCommand_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ev3_interfaces__srv__MotorCommand_Response
    std::shared_ptr<ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ev3_interfaces__srv__MotorCommand_Response
    std::shared_ptr<ev3_interfaces::srv::MotorCommand_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorCommand_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorCommand_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorCommand_Response_

// alias to use template instance with default allocator
using MotorCommand_Response =
  ev3_interfaces::srv::MotorCommand_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ev3_interfaces

namespace ev3_interfaces
{

namespace srv
{

struct MotorCommand
{
  using Request = ev3_interfaces::srv::MotorCommand_Request;
  using Response = ev3_interfaces::srv::MotorCommand_Response;
};

}  // namespace srv

}  // namespace ev3_interfaces

#endif  // EV3_INTERFACES__SRV__DETAIL__MOTOR_COMMAND__STRUCT_HPP_
