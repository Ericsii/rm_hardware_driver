#include <rm_base/buffer_processor_factory.hpp>

#include <rm_base/protocol_types.hpp>
#include "rm_interfaces/msg/robot_command.hpp"

namespace rm_base
{

  typedef struct
  {
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t commd_keyboard;
    uint16_t target_robot_ID;
  } __attribute__((__packed__)) ext_robot_command_t;

  class RobotCommandProcessor : public ProcessInterface
  {
  public:
    explicit RobotCommandProcessor(rclcpp::Node *node)
        : ProcessInterface(node)
    {
      auto topic_name = node_->declare_parameter("robot_command_topic", "robot_command");
      RCLCPP_INFO(node_->get_logger(), "robot_command_topic: %s", topic_name.c_str());
      pub_ = node_->create_publisher<rm_interfaces::msg::RobotCommand>(topic_name,1);
    }

    bool process_packet(const Packet &packet)
    {
      if (std::holds_alternative<rmoss_base::FixedPacket64>(packet))
      {
        auto packet_recv = std::get<rmoss_base::FixedPacket64>(packet);
        rm_interfaces::msg::RobotCommand::UniquePtr msg(new rm_interfaces::msg::RobotCommand());

        ext_robot_command_t data;
        if (!packet_recv.unload_data(data, 2))
        {
          return false;
        }
        msg->header.stamp = node_->get_clock()->now();
        msg->target_position_x = data.target_position_x;
        msg->target_position_y = data.target_position_y;
        msg->target_position_z = data.target_position_z;
        msg->commd_keyboard = data.commd_keyboard;
        msg->target_robot_id = data.target_robot_ID;

        pub_->publish(std::move(msg));
        return true;
      }
      else
      {
        RCLCPP_WARN(node_->get_logger(), "Invalid length of data frame for RobotCommand processor.");
        return false;
      }
    }

  private:
    rclcpp::Publisher<rm_interfaces::msg::RobotCommand>::SharedPtr pub_;
  };

#include <rm_base/register_macro.hpp>

  REGISTER_PROCESSOR_CLASS(RobotCommandProcessor, static_cast<uint8_t>(RecvID::ROBOTCOMMAND))

} // namespace rm_base
