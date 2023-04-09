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
#include <rm_interfaces/msg/robot_bullet_remain.hpp>

#include <rm_base/crc.hpp>
#include <rm_base/buffer_processor_factory.hpp>
#include <rm_base/protocol_types.hpp>

typedef struct
{
  uint16_t bullet_remaining_num_17mm;
  uint16_t bullet_remaining_num_42mm;
  uint16_t coin_remaining_num;
} __attribute__((__packed__)) ext_bullet_remain_t;

TEST(RobotBulletRemainProcessor, test_robot_bullet_remain_processor)
{
  ext_bullet_remain_t bullet_remain;
  bullet_remain.bullet_remaining_num_17mm = 10;
  bullet_remain.bullet_remaining_num_42mm = 10;
  bullet_remain.coin_remaining_num = 100;
  rmoss_base::FixedPacket64 packet;
  packet.load_data(static_cast<uint8_t>(rm_base::RecvID::BULLETREMAINING), 1);
  packet.load_data(bullet_remain, 2);
  packet.set_check_byte(rm_base::Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_robor_bullet_remain_frame");
  EXPECT_TRUE(
    rm_base::ProcessFactory::create(
      static_cast<uint8_t>(rm_base::RecvID::BULLETREMAINING),
      node.get()));
  EXPECT_TRUE(
    rm_base::ProcessFactory::process_packet(
      static_cast<uint8_t>(rm_base::RecvID::
      BULLETREMAINING), packet));
  rclcpp::shutdown();
}
