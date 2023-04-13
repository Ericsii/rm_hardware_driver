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
#include "rm_interfaces/msg/robot_hurt.hpp"

namespace rm_base
{

typedef struct __packed
{
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} __attribute__((__packed__)) ext_robot_hurt_t;

class RobotHurtProcessor : public ProcessInterface
{
public:
  explicit RobotHurtProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("robot_hurt_topic", "robot_hurt");
    RCLCPP_INFO(node_->get_logger(), "robot_hurt_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::RobotHurt>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::RobotHurt::UniquePtr msg(new rm_interfaces::msg::RobotHurt());

      ext_robot_hurt_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->armor_id = data.armor_id;
      msg->hurt_type = data.hurt_type;

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for RobotHurt processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::RobotHurt>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(RobotHurtProcessor, static_cast<uint8_t>(RecvID::ROBOTHURT))

}  // namespace rm_base
