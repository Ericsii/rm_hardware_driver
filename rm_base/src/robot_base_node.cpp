// Copyright 2022 Yunlong Feng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_base/robot_base_node.hpp"

#include "rmoss_base/uart_transporter.hpp"

#include "rm_base/buffer_processor_factory.hpp"
#include "rm_base/crc.hpp"

namespace rm_base
{

namespace serial_send_protocol
{
typedef enum : uint8_t
{
  GimbalCmd = 0x01,
  ChassisCmd = 0x02
} SerialSendProtocol;
}

namespace serial_recv_protocol
{
typedef enum : uint8_t
{
  GameStatus = 0x01
} SerialRecvProtocol;
}

RobotBaseNode::RobotBaseNode(const rclcpp::NodeOptions & options)
: Node("robot_base_node", options)
{
  // declare parameters
  auto port_name = this->declare_parameter("port", "/dev/ttyUSB0");
  int baud_rate = this->declare_parameter("baud_rate", 115200);
  bool enable_realtime_send = this->declare_parameter("enable_realtime_send", false);
  RCLCPP_INFO(this->get_logger(), "Serial port: %s baud_rate: %d", port_name.c_str(), baud_rate);
  RCLCPP_INFO(this->get_logger(), "enable async send: %d", enable_realtime_send);

  RCLCPP_INFO(this->get_logger(), "Initialize serial port.");
  auto transporter = std::make_shared<rmoss_base::UartTransporter>(port_name, baud_rate);
  packet_tool_ = std::make_shared<rmoss_base::FixedPacketTool<64>>(transporter);
  packet_tool_->enable_realtime_send(enable_realtime_send);

  RCLCPP_INFO(this->get_logger(), "Initialize data processors.");
  ProcessFactory::create(serial_recv_protocol::GameStatus, this);

  // task thread
  listen_thread_ = std::make_unique<std::thread>(&RobotBaseNode::listen_loop, this);
}

void RobotBaseNode::listen_loop()
{
  rmoss_base::FixedPacket<64> packet;
  while (rclcpp::ok()) {
    if (packet_tool_->recv_packet(packet)) {
      if (!verify_checksum(packet)) {
        RCLCPP_WARN(this->get_logger(), "CRC8 verify failed.");
        continue;
      }

      uint8_t cmd_id;
      packet.unload_data(cmd_id, 1);
      if (!ProcessFactory::process_packet(cmd_id, packet)) {
        RCLCPP_WARN(this->get_logger(), "No such frame cmd_key: 0x%x", cmd_id);
      }
    }
  }
}

bool RobotBaseNode::verify_checksum(const rmoss_base::FixedPacket<64> & packet)
{
  auto expected = *(packet.buffer() + 62);
  return Get_CRC8_Check_Sum(packet.buffer() + 1, 61, CRC8_INIT) == expected;
}

bool RobotBaseNode::checksum_send(rmoss_base::FixedPacket<64> & packet)
{
  // 数据段长度 61 字节
  packet.set_check_byte(Get_CRC8_Check_Sum(packet.buffer() + 1, 61, CRC8_INIT));
  return packet_tool_->send_packet(packet);
}

void RobotBaseNode::gimbal_cmd_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  // | 0x01 | tid 1byte | yaw_type 1byte | pitch_type 1byte | position.yaw 4byte | position.pitch 4byte | velocity.yaw 4byte | velocity.pitch 4byte |
  // 4byte | velocity.pitch 4byte |
  rmoss_base::FixedPacket<64> packet;
  packet.load_data(serial_send_protocol::GimbalCmd, 1);
  packet.load_data(msg->tid, 2);
  packet.load_data(msg->yaw_type, 3);
  packet.load_data(msg->pitch_type, 4);
  packet.load_data(msg->position, 5);
  packet.load_data(msg->velocity, 13);
  if (!this->checksum_send(packet)) {
    RCLCPP_WARN(this->get_logger(), "Send GimbalCmd failed.");
  }
}

void RobotBaseNode::chassis_cmd_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
  // | 0x02 | tid 1byte | type 1byte | twist.linear.x 4byte | twist.linear.y 4byte | accel.linear.x 4byte |
  // accel.linear.y 4byte | accel.anglular 4byte |
  rmoss_base::FixedPacket<64> packet;
  packet.load_data(serial_send_protocol::ChassisCmd, 1);
  packet.load_data(msg->tid, 2);
  packet.load_data(msg->type, 3);
  packet.load_data(static_cast<float>(msg->twist.linear.x), 4);
  packet.load_data(static_cast<float>(msg->twist.linear.y), 8);
  packet.load_data(static_cast<float>(msg->twist.angular.z), 12);
  packet.load_data(static_cast<float>(msg->accel.linear.x), 16);
  packet.load_data(static_cast<float>(msg->accel.linear.y), 20);
  packet.load_data(static_cast<float>(msg->accel.angular.z), 24);

  if (!this->checksum_send(packet)) {
    RCLCPP_WARN(this->get_logger(), "Send ChassisCmd failed.");
  }
}

}  // namespace rm_base

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_base::RobotBaseNode);
