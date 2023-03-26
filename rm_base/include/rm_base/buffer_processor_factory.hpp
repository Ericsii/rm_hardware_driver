// Copyright 2022 Yunlong Feng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_BASE__BUFFER_PROCESSOR_FACTORY_HPP_
#define RM_BASE__BUFFER_PROCESSOR_FACTORY_HPP_

#include <string>
#include <functional>
#include <unordered_map>
#include <variant>

#include "rclcpp/rclcpp.hpp"

#include "rmoss_base/fixed_packet_tool.hpp"

namespace rm_base
{

class ProcessInterface
{
  /**
   * @brief 消息处理基类
   * 不同的消息帧重载 process_packet 来解析消息帧
   */

public:
  using Packet = std::variant<rmoss_base::FixedPacket16, rmoss_base::FixedPacket32,
      rmoss_base::FixedPacket64>;

public:
  ProcessInterface(rclcpp::Node * node)
  : node_(node)
  {
  }

  /**
   * @brief 帧处理函数
   * 解析 packet 帧
   * @param packet 完整的帧，类型为 rmoss_base::FixedPacket16/rmoss_base::FixedPacket32/rmoss_base::FixedPacket64
   * @return true 处理成功
   * @return false 处理失败
   */
  virtual bool process_packet(const Packet & packet) = 0;

  RCLCPP_SHARED_PTR_DEFINITIONS(ProcessInterface)

protected:
  rclcpp::Node * node_;
};

class ProcessFactory
{
  /**
   * @brief 串口处理工厂类
   * 根据不同的key(消息包id)来生成对应的处理对象
   */

public:
  using CreateFunction = std::function<ProcessInterface::SharedPtr(rclcpp::Node * node)>;

  /**
   * @brief 注册处理类的派生类构造函数
   *
   * @tparam T
   * @param key
   * @return true
   * @return false
   */
  template<typename T>
  static bool register_class(const uint8_t & key)
  {
    if (auto it = get_creator_map().find(key); it == get_creator_map().end()) {
      get_creator_map()[key] = [](rclcpp::Node * node) {return std::make_shared<T>(node);};
      return true;
    }
    return false;
  }

  /**
   * @brief 根据 Key 创建新的帧处理对象，并添加进处理
   *
   * @param key 帧标志id
   * @param node 处理帧的ROS节点
   * @return true
   * @return false
   */
  static bool create(const uint8_t & key, rclcpp::Node * node)
  {
    auto processor = get_processor_map().find(key);
    auto creator = get_creator_map().find(key);
    if (processor == get_processor_map().end() && creator != get_creator_map().end()) {
      get_processor_map()[key] = creator->second(node);
      return true;
    }
    return false;
  }

  /**
   * @brief 解析具体消息帧
   *
   * @param key 帧标志id
   * @param packet 完整的帧内容
   * @return true
   * @return false
   */
  static bool process_packet(const uint8_t & key, const ProcessInterface::Packet & packet)
  {
    if (auto it = get_processor_map().find(key); it != get_processor_map().end()) {
      return it->second->process_packet(packet);
    }
    return false;
  }

private:
  static std::unordered_map<uint8_t, CreateFunction> & get_creator_map()
  {
    static std::unordered_map<uint8_t, CreateFunction> creator_map;
    return creator_map;
  }

  static std::unordered_map<uint8_t, ProcessInterface::SharedPtr> & get_processor_map()
  {
    static std::unordered_map<uint8_t, ProcessInterface::SharedPtr> processor_map;
    return processor_map;
  }
};

// 自动注册辅助模板类
template<typename T>
struct ProcessorRigisterHelper
{
};

#define PROCESSOR_REGISTER(Class, Key) \
  template<> \
  struct ProcessorRigisterHelper<Class> \
  { \
    ProcessorRigisterHelper<Class>() \
    { \
      is_registed = ProcessFactory::register_class<Class>(Key); \
    } \
    bool is_registed; \
  }; \
  static ProcessorRigisterHelper<Class> register_plugin_ ## Class;

}  // namespace rm_base

#endif  // RM_BASE__BUFFER_PROCESSOR_FACTORY_HPP_
