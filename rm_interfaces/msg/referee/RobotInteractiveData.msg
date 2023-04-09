# msg for referee system
# robot interactive data
# 0x0200~0x02FF 可以在以上 ID 段选取, 具体 ID 含义由参赛队自定义
uint16 data_id
# 机器人 ID: 1, 英雄(红); 2, 工程(红); 3/4/5, 步兵(红); 6, 空中(红); 7, 哨兵(红); 9, 雷达站红;
# 101, 英雄(蓝); 102, 工程(蓝); 103/104/105, 步兵(蓝); 106, 空中(蓝); 107, 哨兵(蓝);  109, 雷达站蓝
uint16 sender_id
uint16 receiver_id

# 由于串口通讯帧限制最大长度 64Byte 限制, data 段最大 49Byte
uint8[54] data