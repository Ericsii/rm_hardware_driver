#include <rm_base/buffer_processor_factory.hpp>

namespace rm_base
{

bool ProcessFactory::create(const uint8_t & key, rclcpp::Node * node)
{
  auto processor = get_processor_map().find(key);
  auto creator = get_creator_map().find(key);
  if (processor == get_processor_map().end() && creator != get_creator_map().end()) {
    get_processor_map()[key] = creator->second(node);
    return true;
  }
  return false;
}

bool ProcessFactory::process_packet(const uint8_t & key, const ProcessInterface::Packet & packet)
{
  if (auto it = get_processor_map().find(key); it != get_processor_map().end()) {
    return it->second->process_packet(packet);
  }
  return false;
}

std::vector<ProcessInterface::SharedPtr> ProcessFactory::get_processors()
{
  std::vector<ProcessInterface::SharedPtr> processors;
  for (auto & processor : get_processor_map()) {
    processors.push_back(processor.second);
  }
  return processors;
}

std::vector<uint8_t> ProcessFactory::get_creator_keys()
{
  std::vector<uint8_t> keys;
  for (auto & processor : get_creator_map()) {
    keys.push_back(processor.first);
  }
  return keys;
}

std::unordered_map<uint8_t, ProcessFactory::CreateFunction> & ProcessFactory::get_creator_map()
{
  static std::unordered_map<uint8_t, CreateFunction> creator_map;
  return creator_map;
}

std::unordered_map<uint8_t, ProcessInterface::SharedPtr> & ProcessFactory::get_processor_map()
{
  static std::unordered_map<uint8_t, ProcessInterface::SharedPtr> processor_map;
  return processor_map;
}

}  // namespace rm_base
