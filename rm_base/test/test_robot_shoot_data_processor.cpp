#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/robot_shoot_data.hpp>

#include <rm_base/crc.hpp>
#include <rm_base/buffer_processor_factory.hpp>
#include <rm_base/protocol_types.hpp>

#include <iostream>

typedef struct
{
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
} __attribute__((__packed__)) ext_shoot_data_t;

TEST(RobotShootDataProcessor, test_shoot_data_processor)
{
  ext_shoot_data_t shoot_data;
  shoot_data.bullet_type = 1;
  shoot_data.shooter_id = 1;
  shoot_data.bullet_freq = 18.1;
  rmoss_base::FixedPacket64 packet;
  packet.load_data(static_cast<uint8_t>(rm_base::RecvID::SHOOTDATA), 1);
  packet.load_data(shoot_data, 2);
  packet.set_check_byte(rm_base::Get_CRC8_Check_Sum(packet.buffer() + 1, 61, rm_base::CRC8_INIT));

  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_shoot_data_frame");
  EXPECT_TRUE(
    rm_base::ProcessFactory::create(
      static_cast<uint8_t>(rm_base::RecvID::SHOOTDATA),
      node.get()));
  EXPECT_TRUE(
    rm_base::ProcessFactory::process_packet(
      static_cast<uint8_t>(rm_base::RecvID::
      SHOOTDATA), packet));
  rclcpp::shutdown();
}
