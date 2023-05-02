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
#include "rm_interfaces/msg/game_robot_pos.hpp"

namespace rm_base
{

typedef struct __packed
{
  float x;
  float y;
  float z;
  float yaw;
} __attribute__((__packed__)) ext_game_robot_pos_t;

class GameRobotPosProcessor : public ProcessInterface
{
public:
  explicit GameRobotPosProcessor(rclcpp::Node * node)
  : ProcessInterface(node)
  {
    auto topic_name = node_->declare_parameter("game_robot_pos_topic", "game_robot_pos");
    RCLCPP_INFO(node_->get_logger(), "game_robot_pos_topic: %s", topic_name.c_str());
    pub_ = node_->create_publisher<rm_interfaces::msg::GameRobotPos>(topic_name, 10);
  }

  bool process_packet(const Packet & packet)
  {
    if (std::holds_alternative<rmoss_base::FixedPacket64>(packet)) {
      auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
      rm_interfaces::msg::GameRobotPos::UniquePtr msg(new rm_interfaces::msg::GameRobotPos());
      msg->header.frame_id = "base_link";
      msg->header.stamp = node_->get_clock()->now();

      ext_game_robot_pos_t data;
      if (!packet_recv.unload_data(data, 2)) {
        return false;
      }
      msg->x = data.x;
      msg->y = data.y;
      msg->z = data.z;
      msg->yaw = data.yaw;

      pub_->publish(std::move(msg));
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for GameRobotPos processor.");
      return false;
    }
  }

private:
  rclcpp::Publisher<rm_interfaces::msg::GameRobotPos>::SharedPtr pub_;
};

#include <rm_base/register_macro.hpp>

REGISTER_PROCESSOR_CLASS(GameRobotPosProcessor, static_cast<uint8_t>(RecvID::GAMEROBOTPOS))

}  // namespace rm_base
