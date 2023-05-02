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
#include "rm_interfaces/msg/robot_bullet_remain.hpp"

namespace rm_base
{

typedef struct
{
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
} __attribute__((__packed__)) ext_bullet_remain_t;

class RobotBulletRemainProcessor : public ProcessInterface
{
public:
  explicit RobotBulletRemainProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("robot_bullet_remain_topic", "robot_bullet_remain");
    RCLCPP_INFO(node_->get_logger(), "robot_bullet_remain_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::RobotBulletRemain>(
      topic_name,
      10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::RobotBulletRemain::UniquePtr msg(
        new rm_interfaces::msg::RobotBulletRemain());

      ext_bullet_remain_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->bullet_remaining_num_17mm = data.bullet_remaining_num_17mm;
      msg->bullet_remaining_num_42mm = data.bullet_remaining_num_42mm;
      msg->coin_remaining_num = data.coin_remaining_num;

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "Invalid length of data frame for RobotBulletRemain processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::RobotBulletRemain>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(RobotBulletRemainProcessor, static_cast<uint8_t>(RecvID::BULLETREMAINING))

}  // namespace rm_base
