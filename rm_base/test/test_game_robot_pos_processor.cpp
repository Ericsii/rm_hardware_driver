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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/game_robot_pos.hpp>

#include <rm_base/crc.hpp>
#include <rm_base/buffer_processor_factory.hpp>
#include <rm_base/protocol_types.hpp>

typedef struct __packed
{
  float x;
  float y;
  float z;
  float yaw;
} __attribute__((__packed__)) ext_game_robot_pos_t;

TEST(GameRobotPosProcessor, test_game_robot_pos_processor)
{
  ext_game_robot_pos_t game_robot_pos;
  game_robot_pos.x = 0.0;
  game_robot_pos.y = 0.0;
  game_robot_pos.z = 0.0;
  game_robot_pos.yaw = 0.0;
  rmoss_base::FixedPacket64 packet;
  packet.load_data(static_cast<uint8_t>(rm_base::RecvID::GAMEROBOTPOS), 1);
  packet.load_data(game_robot_pos, 2);
  packet.set_check_byte(rm_base::Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_game_robot_pos_frame");
  EXPECT_TRUE(
    rm_base::ProcessFactory::create(
      static_cast<uint8_t>(rm_base::RecvID::GAMEROBOTPOS),
      node.get()));
  EXPECT_TRUE(
    rm_base::ProcessFactory::process_packet(
      static_cast<uint8_t>(rm_base::RecvID::
      GAMEROBOTPOS), packet));
  rclcpp::shutdown();
}
