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

namespace rm_base
{

namespace serial_protocol
{
typedef enum : uint8_t
{
  GimbalCmd = 0x01,
  ChassisCmd = 0x02
} SerialProtocol;
}

RobotBaseNode::RobotBaseNode(const rclcpp::NodeOptions & options)
: Node("robot_base_node", options)
{
  // declare parameters
  auto port_name = this->declare_parameter("port", "/dev/ttyUSB0");
  int baud_rate = this->declare_parameter("baud_rate", 115200);
  bool enable_realtime_send = this->declare_parameter("enable_realtime_send", false);

  auto transporter = std::make_shared<rmoss_base::UartTransporter>(port_name, baud_rate);
  packet_tool_ = std::make_shared<rmoss_base::FixedPacketTool<32>>(transporter);
  packet_tool_->enable_realtime_send(enable_realtime_send);

  // task thread
  listen_thread_ = std::make_unique<std::thread>(&RobotBaseNode::listen_loop, this);
}

void RobotBaseNode::listen_loop()
{
  rmoss_base::FixedPacket<32> packet;
  while (rclcpp::ok()) {
    if (packet_tool_->recv_packet(packet)) {
      uint8_t cmd_id;
      packet.unload_data(cmd_id, 1);
      if (!ProcessFactory::process_packet(cmd_id, packet)) {
        RCLCPP_WARN(this->get_logger(), "No such frame cmd_key: 0x%x", cmd_id);
      }
    }
  }
}

void RobotBaseNode::gimbal_cmd_cb(const rmoss_interfaces::msg::GimbalCmd::SharedPtr msg)
{
  rmoss_base::FixedPacket<32> packet;
  packet.load_data(serial_protocol::GimbalCmd, 1);
  packet.load_data(msg->yaw_type, 2);
  packet.load_data(msg->pitch_type, 3);
  packet.load_data(msg->position, 4);
  packet.load_data(msg->velocity, 12);
  if (!packet_tool_->send_packet(packet)) {
    RCLCPP_WARN(this->get_logger(), "Send GimbalCmd failed.");
  }
}

void RobotBaseNode::chassis_cmd_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
  rmoss_base::FixedPacket<32> packet;
  packet.load_data(serial_protocol::ChassisCmd, 1);

  if (!packet_tool_->send_packet(packet)) {
    RCLCPP_WARN(this->get_logger(), "Send ChassisCmd failed.");
  }
}

}  // namespace rm_base

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_base::RobotBaseNode);
