#ifndef RM_BASE__PROCESSOR_INTERFACE_HPP_
#define RM_BASE__PROCESSOR_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <rmoss_base/fixed_packet.hpp>

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
}  // namespace rm_base

#endif  // RM_BASE__PROCESSOR_INTERFACE_HPP_
