// Copyright 2023 Tingxu Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_base/buffer_processor_factory.hpp"

#include <rm_base/protocol_types.hpp>
#include "rm_interfaces/msg/robot_buff.hpp"

namespace rm_base
{

typedef struct __packed
{
  uint8_t power_rune_buff;
} __attribute__((__packed__)) ext_buff_t;

class RobotBuffProcessor : public ProcessInterface
{
public:
  explicit RobotBuffProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("robor_buff_topic", "robor_buff");
    RCLCPP_INFO(node_->get_logger(), "robor_buff_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::RobotBuff>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::RobotBuff::UniquePtr msg(new rm_interfaces::msg::RobotBuff());

      ext_buff_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->healing = get_buff_data(data, 0);
      msg->cooling = get_buff_data(data, 1);
      msg->defending = get_buff_data(data, 2);
      msg->attacking = get_buff_data(data, 3);

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for RobotBuff processor.");
      return false;
    }
  }

private:
  bool get_buff_data(ext_buff_t data, int bit)
  {
    return (data.power_rune_buff >> bit) & 0x0001;
  }

  rclcpp::Publisher<rm_interfaces::msg::RobotBuff>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(RobotBuffProcessor, static_cast<uint8_t>(RecvID::BUFFMUSK))

}  // namespace rm_base
