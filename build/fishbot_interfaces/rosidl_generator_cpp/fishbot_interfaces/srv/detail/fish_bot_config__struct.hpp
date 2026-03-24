// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from fishbot_interfaces:srv/FishBotConfig.idl
// generated code does not contain a copyright notice

#ifndef FISHBOT_INTERFACES__SRV__DETAIL__FISH_BOT_CONFIG__STRUCT_HPP_
#define FISHBOT_INTERFACES__SRV__DETAIL__FISH_BOT_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__fishbot_interfaces__srv__FishBotConfig_Request __attribute__((deprecated))
#else
# define DEPRECATED__fishbot_interfaces__srv__FishBotConfig_Request __declspec(deprecated)
#endif

namespace fishbot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FishBotConfig_Request_
{
  using Type = FishBotConfig_Request_<ContainerAllocator>;

  explicit FishBotConfig_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key = "";
      this->value = "";
    }
  }

  explicit FishBotConfig_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : key(_alloc),
    value(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key = "";
      this->value = "";
    }
  }

  // field types and members
  using _key_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _key_type key;
  using _value_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _value_type value;

  // setters for named parameter idiom
  Type & set__key(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->key = _arg;
    return *this;
  }
  Type & set__value(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fishbot_interfaces__srv__FishBotConfig_Request
    std::shared_ptr<fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fishbot_interfaces__srv__FishBotConfig_Request
    std::shared_ptr<fishbot_interfaces::srv::FishBotConfig_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FishBotConfig_Request_ & other) const
  {
    if (this->key != other.key) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const FishBotConfig_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FishBotConfig_Request_

// alias to use template instance with default allocator
using FishBotConfig_Request =
  fishbot_interfaces::srv::FishBotConfig_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace fishbot_interfaces


#ifndef _WIN32
# define DEPRECATED__fishbot_interfaces__srv__FishBotConfig_Response __attribute__((deprecated))
#else
# define DEPRECATED__fishbot_interfaces__srv__FishBotConfig_Response __declspec(deprecated)
#endif

namespace fishbot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FishBotConfig_Response_
{
  using Type = FishBotConfig_Response_<ContainerAllocator>;

  explicit FishBotConfig_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key = "";
      this->value = "";
    }
  }

  explicit FishBotConfig_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : key(_alloc),
    value(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->key = "";
      this->value = "";
    }
  }

  // field types and members
  using _key_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _key_type key;
  using _value_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _value_type value;

  // setters for named parameter idiom
  Type & set__key(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->key = _arg;
    return *this;
  }
  Type & set__value(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__fishbot_interfaces__srv__FishBotConfig_Response
    std::shared_ptr<fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__fishbot_interfaces__srv__FishBotConfig_Response
    std::shared_ptr<fishbot_interfaces::srv::FishBotConfig_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FishBotConfig_Response_ & other) const
  {
    if (this->key != other.key) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const FishBotConfig_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FishBotConfig_Response_

// alias to use template instance with default allocator
using FishBotConfig_Response =
  fishbot_interfaces::srv::FishBotConfig_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace fishbot_interfaces

namespace fishbot_interfaces
{

namespace srv
{

struct FishBotConfig
{
  using Request = fishbot_interfaces::srv::FishBotConfig_Request;
  using Response = fishbot_interfaces::srv::FishBotConfig_Response;
};

}  // namespace srv

}  // namespace fishbot_interfaces

#endif  // FISHBOT_INTERFACES__SRV__DETAIL__FISH_BOT_CONFIG__STRUCT_HPP_
