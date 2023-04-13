// Copyright 2023 Yunlong Feng
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

#include <rm_base/buffer_processor_factory.hpp>

#include <rm_base/protocol_types.hpp>
#include <rm_interfaces/msg/robot_shoot_data.hpp>

namespace rm_base
{

typedef struct
{
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
} __attribute__((__packed__)) ext_shoot_data_t;

class RobotShootDataProcessor : public ProcessInterface
{
public:
  explicit RobotShootDataProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("robot_shoot_data_topic", "robot_shoot_data");
    RCLCPP_INFO(node_->get_logger(), "robot_shoot_data topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::RobotShootData>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::RobotShootData::UniquePtr msg(new rm_interfaces::msg::RobotShootData());

      ext_shoot_data_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->bullet_type = data.bullet_type;
      msg->shooter_id = data.shooter_id;
      msg->bullet_freq = data.bullet_freq;
      msg->bullet_speed = data.bullet_speed;

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "Invalid length of data frame for RobotShootData processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::RobotShootData>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(RobotShootDataProcessor, static_cast<uint8_t>(RecvID::SHOOTDATA))

}  // namespace rm_base
