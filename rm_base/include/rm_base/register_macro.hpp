#include <rm_base/buffer_processor_factory.hpp>

#define REGISTER_PROCESSOR_CLASS(class_name, key) \
  static bool class_name ## _registered = \
    rm_base::ProcessFactory::register_class<class_name>(key);
