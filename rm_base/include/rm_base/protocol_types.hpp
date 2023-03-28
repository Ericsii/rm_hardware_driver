#ifndef RM_BASE__PROTOCOL_TYPES_HPP_
#define RM_BASE__PROTOCOL_TYPES_HPP_

#include <cstdint>

namespace rm_base
{

enum class RecvID : uint8_t
{
  GAMESTATUS = 0x01,
  GAMERESULT = 0x02,
  ROBOTHP = 0x03,
  EVENTDATA = 0x04,
  ROBOTSTATUS = 0x05,
  POWERHEATDATA = 0x06,
  GAMEROBOTPOS = 0x07,
  BUFFMUSK = 0x08,
  ROBOTHURT = 0x09,
  SHOOTDATA = 0x10,
  BULLETREMAINING = 0x11,
  REFREEINTERACT = 0x12,
  ROBOTJOINTSTATE = 0x13
};

enum class SendID : uint8_t
{
  CHASSISCMD = 0x01,
  GIMBALCMD = 0x02,
  SHOOTCMD = 0x03,
  REFREEINTERACT = 0x04
};

}  // namespace rm_base

#endif  // RM_BASE__PROTOCOL_TYPES_HPP_
