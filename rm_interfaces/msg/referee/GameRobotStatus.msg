# msg for referee system
# game robot status

# 本机器人 ID:
#   1: 红方英雄机器人
#   2: 红方工程机器人
#   3/4/5: 红方步兵机器人
#   6: 红方空中机器人
#   7: 红方哨兵机器人
#   8: 红方飞镖机器人
#   9: 红方雷达站
#   101: 蓝方英雄机器人
#   102: 蓝方工程机器人
#   103/104/105: 蓝方步兵机器人
#   106: 蓝方空中机器人
#   107: 蓝方哨兵机器人
#   108: 蓝方飞镖机器人
#   109: 蓝方雷达站
uint8 robot_id
uint8 robot_level # 机器人等级: 1: 一级 2: 二级 3: 三级
uint16 remain_hp # 机器人剩余血量
uint16 max_hp # 机器人上限血量
uint16 shooter_id1_17mm_cooling_rate # 机器人 1 号 17mm 枪口每秒冷却值
uint16 shooter_id1_17mm_cooling_limit # 机器人 1 号 17mm 枪口每秒冷却值
uint16 shooter_id1_17mm_speed_limit # 机器人 1 号 17mm 枪口上限速度 单位 m/s
uint16 shooter_id2_17mm_cooling_rate # 机器人 2 号 17mm 枪口每秒冷却值
uint16 shooter_id2_17mm_cooling_limit # 机器人 2 号 17mm 枪口热量上限
uint16 shooter_id2_17mm_speed_limit # 机器人 2 号 17mm 枪口上限速度 单位 m/s
uint16 shooter_id1_42mm_cooling_rate # 机器人 42mm 枪口每秒冷却值
uint16 shooter_id1_42mm_cooling_limit # 机器人 42mm 枪口热量上限
uint16 shooter_id1_42mm_speed_limit # 机器人 42mm 枪口上限速度 单位 m/s
uint16 chassis_power_limit # 机器人底盘功率限制上限
# 主控电源输出情况：
# 0 bit: gimbal 口输出:  1 为有 24V 输出, 0 为无 24v 输出
# 1 bit: chassis 口输出: 1 为有 24V 输出, 0 为无 24v 输出
# 2 bit: shooter 口输出: 1 为有 24V 输出, 0 为无 24v 输出
bool gimbal_power_enable
bool chassis_power_enables
bool shooter_power_enable