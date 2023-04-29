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
#include <rm_interfaces/msg/robot_command.hpp>

#include <rm_base/crc.hpp>
#include <rm_base/buffer_processor_factory.hpp>
#include <rm_base/protocol_types.hpp>

typedef struct
{
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;
    uint16_t target_robot_ID;
} __attribute__((__packed__)) ext_robot_command_t;


TEST(RobotCommandProcessor, test_robot_command_processor)
{
  ext_robot_command_t robot_command;
  robot_command.target_position_x = 1.0;
  robot_command.target_position_y = 2.0;
  robot_command.target_position_z = 3.0;
  robot_command.commd_keyboard = 0;
  robot_command.target_robot_ID = 7;
  rmoss_base::FixedPacket64 packet;
  packet.load_data(static_cast<uint8_t>(rm_base::RecvID::ROBOTCOMMAND), 1);
  packet.load_data(robot_command, 2);
  packet.set_check_byte(rm_base::Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_robot_command_frame");
  EXPECT_TRUE(
    rm_base::ProcessFactory::create(
      static_cast<uint8_t>(rm_base::RecvID::ROBOTCOMMAND),
      node.get()));
  EXPECT_TRUE(
    rm_base::ProcessFactory::process_packet(
      static_cast<uint8_t>(rm_base::RecvID::
      ROBOTCOMMAND), packet));
  rclcpp::shutdown();
}
