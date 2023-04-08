# msg for referee system
# Robot hurt

uint8 ARMOR_HURT = 0 # 装甲伤害扣血
uint8 MODULE_OFFLINE_HURT = 1 # 模块掉线扣血s
uint8 SHOOT_OVERSPEED_HURT = 2 # 超射速扣血
uint8 BULLET_OVERSPEED_HURT = 3 # 超枪口热量扣血
uint8 CHASSIS_OVERSPEED_HURT = 4 # 超底盘功率扣血
uint8 ARMOR_HIT_HURT = 5 # 装甲撞击扣血

uint8 armor_id # 当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0
uint8 hurt_type # 伤害类型，0x0 装甲伤害扣血；
# 0x1 模块掉线扣血；
# 0x2 超射速扣血；
# 0x3 超枪口热量扣血；
# 0x4 超底盘功率扣血；
# 0x5 装甲撞击扣血