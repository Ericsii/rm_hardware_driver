# msg for referee system
# game status
std_msgs/Header header

uint8 game_type_progress # 0-3bit:game_type | 4-7bit:game_progress
uint16 stage_remain_time # Remain time of current stage (s)

uint64 sync_time_stamp # Unix time stamp. Only used when valid NTP server provided to MCU