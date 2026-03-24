// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from fishbot_interfaces:srv/FishBotConfig.idl
// generated code does not contain a copyright notice

#ifndef FISHBOT_INTERFACES__SRV__DETAIL__FISH_BOT_CONFIG__BUILDER_HPP_
#define FISHBOT_INTERFACES__SRV__DETAIL__FISH_BOT_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "fishbot_interfaces/srv/detail/fish_bot_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace fishbot_interfaces
{

namespace srv
{

namespace builder
{

class Init_FishBotConfig_Request_value
{
public:
  explicit Init_FishBotConfig_Request_value(::fishbot_interfaces::srv::FishBotConfig_Request & msg)
  : msg_(msg)
  {}
  ::fishbot_interfaces::srv::FishBotConfig_Request value(::fishbot_interfaces::srv::FishBotConfig_Request::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fishbot_interfaces::srv::FishBotConfig_Request msg_;
};

class Init_FishBotConfig_Request_key
{
public:
  Init_FishBotConfig_Request_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FishBotConfig_Request_value key(::fishbot_interfaces::srv::FishBotConfig_Request::_key_type arg)
  {
    msg_.key = std::move(arg);
    return Init_FishBotConfig_Request_value(msg_);
  }

private:
  ::fishbot_interfaces::srv::FishBotConfig_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::fishbot_interfaces::srv::FishBotConfig_Request>()
{
  return fishbot_interfaces::srv::builder::Init_FishBotConfig_Request_key();
}

}  // namespace fishbot_interfaces


namespace fishbot_interfaces
{

namespace srv
{

namespace builder
{

class Init_FishBotConfig_Response_value
{
public:
  explicit Init_FishBotConfig_Response_value(::fishbot_interfaces::srv::FishBotConfig_Response & msg)
  : msg_(msg)
  {}
  ::fishbot_interfaces::srv::FishBotConfig_Response value(::fishbot_interfaces::srv::FishBotConfig_Response::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::fishbot_interfaces::srv::FishBotConfig_Response msg_;
};

class Init_FishBotConfig_Response_key
{
public:
  Init_FishBotConfig_Response_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FishBotConfig_Response_value key(::fishbot_interfaces::srv::FishBotConfig_Response::_key_type arg)
  {
    msg_.key = std::move(arg);
    return Init_FishBotConfig_Response_value(msg_);
  }

private:
  ::fishbot_interfaces::srv::FishBotConfig_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::fishbot_interfaces::srv::FishBotConfig_Response>()
{
  return fishbot_interfaces::srv::builder::Init_FishBotConfig_Response_key();
}

}  // namespace fishbot_interfaces

#endif  // FISHBOT_INTERFACES__SRV__DETAIL__FISH_BOT_CONFIG__BUILDER_HPP_
